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

bool new_data_available = false;
uint8_t spi_frame[256];
unsigned int sensor_value_counter = 0;

WIFI_LOVE *whylove;

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
    Serial.begin(115200);

    Serial.println("hi there");

    IPAddress broadcastIP(10,25,15,255);
    whylove = new WIFI_LOVE ("1-UTUM-Guest", "", broadcastIP);

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

    whylove->checkHostConfig();
    LOG(logINFO, "received config");

    /************** SET UP SPI SLAVE OF FPGA*****************/
    SPISlave.onData([](uint8_t * data, size_t len) {
//        Serial.printf("received data:\n");
//        char str[33];
//        for(uint i=0;i<len;i+=4){
//          uint32_t val = uint32_t(data[i+3]<<24|data[i+2]<<16|data[i+1]<<8|data[i]);
//          Serial.printf("%d:\t",i);
//          printBits(val);
//          Serial.println();
//          Serial.printf("   \t%d\t%d\t%d\t%d\t\n", data[i+3], data[i+2], data[i+1], data[i] );
//          
//        }
        sensor_value_counter++;
        whylove->fmsgSensorDataT_s( data, sizeof(uint8_t)*len);
    });

    // Setup SPI Slave registers and pins
    SPISlave.begin();
}

void loop() {
    Serial.println(sensor_value_counter);
    delay(1000);
  }
