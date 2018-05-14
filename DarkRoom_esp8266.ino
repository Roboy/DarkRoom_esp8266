/*
    DarkRoom SPI slave
    Connect the SPI Master device to the following pins on the esp8266:
    GPIO    NodeMCU   Name  |   Uno
  ===================================
     15       D8       SS   |   D10
     13       D7      MOSI  |   D11
     12       D6      MISO  |   D12
     14       D5      SCK   |   D13
    Note: If the ESP is booting at a moment when the SPI Master has the Select line HIGH (deselected)
    the ESP8266 WILL FAIL to boot!
    See SPISlave_SafeMaster example for possible workaround
*/

#include "SPISlave.h"
#include "wirelessLove.h"
#include "fifo.h"
#include "logging.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

enum COMMAND{
    TRACKING = 1,
    MPU = 2,
    SENSOR = 3,
    RESET = 4
};

bool new_data_available = false;
uint8_t spi_frame[256];
unsigned int sensor_value_counter = 0, sensor_value_counter_prev = 0;

WIFI_LOVE *whylove;
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void printBits(uint32_t val){
  int i=0;
 for(uint32_t mask = 0x80000000; mask; mask >>= 1){
    if(i%8==0)
         Serial.print(' ');
    i++;
    if(mask  & val)
       Serial.print('1');
    else
       Serial.print('0');
   
 }
}

void setup()
{
    // set trigger pin
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
  
    Serial.begin(115200);

    Serial.println("");
    Serial.println("------------------------------------------------------");
    Serial.println("                    DARKROOM ESP");
    Serial.println("------------------------------------------------------");


    IPAddress broadcastIP(192,168,255,255);
    whylove = new WIFI_LOVE ("roboy", "wiihackroboy", broadcastIP);

    /************** WIFI *****************************/
    if(ES_WIFI_SUCCESS != whylove->initWifi()){
        Serial.println("Error in initializing the WiFi!"); 
    }else{
        whylove->printWifiStatus(); 
    }

    if(ES_WIFI_SUCCESS != whylove->initUDPSockets()){
        Serial.println("Error in initializing the UDP Sockets!"); 
    }

    enableLogging = false;   

//    whylove->checkHostConfig();
//    LOG(logINFO, "received config");

    /************** SET UP SPI SLAVE OF FPGA*****************/
    SPISlave.onData([](uint8_t * data, size_t len) {
        if(sensor_value_counter%100==0){
          Serial.printf("received %d sensor frames, this frame:\n", sensor_value_counter);
          char str[33];
          for(uint i=0;i<len;i+=4){
            uint32_t val = uint32_t(data[i+3]<<24|data[i+2]<<16|data[i+1]<<8|data[i]);
            Serial.printf("%d:\t",i);
            printBits(val);
            Serial.println();
            Serial.printf("   \t%d\t%d\t%d\t%d\t\n", data[i+3], data[i+2], data[i+1], data[i] );
            
          }
        }
        sensor_value_counter++;
        whylove->fmsgSensorDataT_s( data, sizeof(uint8_t)*len);
    });

//     SPISlave.onStatusSent([]() {
//        Serial.println("Status Read By FPGA");
//    });

    // Setup SPI Slave registers and pins
    SPISlave.begin();

//    while(true){
//      delay(1000);
//      Serial.println(sensor_value_counter);
//    }
    /************** SET UP MPU6050 *****************/
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    do {
        if(whylove->receiveCommand()){
          // this command will be read by the fpga regularily
          SPISlave.setStatus(whylove->command);
          switch(whylove->command & 0xF){
            case RESET:
              Serial.println("rebooting ESP in 3 seconds");
              delay(3000);
              ESP.reset();
              break;
            case MPU:
              Serial.printf("Received MPU6050 toggle %s", (whylove->command>>4)?"true":"false");
              dmpReady = whylove->command>>4;
              break;
            case TRACKING:
              Serial.printf("Received Tracking toggle %s", (whylove->command>>4)?"true":"false");
              break;
          }
        }
        yield();
    }while (!mpuInterrupt && fifoCount < packetSize);

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);

        whylove->sendImuData(q,aa,gravity);
    }
  }
