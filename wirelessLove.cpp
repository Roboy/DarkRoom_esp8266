#include <SPI.h>
#include "wirelessLove.h"


 const char  TestBuffer[] ="Hello World"; 
 const int    timestampSize = 2; 

WIFI_LOVE::WIFI_LOVE(const char* SSID, const char* PASSWD, IPAddress &broadcastIP): 
broadcastIP(broadcastIP){
    sprintf(pass,PASSWD);
    sprintf(ssid,SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWD);

//    hostIP = IPAddress(10,25,12,189);
}

int WIFI_LOVE::printWifiStatus(void)
{
    Serial.print("SSID: "); 
    Serial.println(WiFi.SSID()); 

    IPAddress ip = WiFi.localIP(); 
    printIP(ip);

    long rssi = WiFi.RSSI(); 
    Serial.print("signal strength (RSSI):"); 
    Serial.print(rssi); 
    Serial.println(" dBm"); 

    return (int) ES_WIFI_SUCCESS; 
}

int WIFI_LOVE::initWifi(void)
{
    uint32_t timoutCounter = 0; 

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    printWifiStatus();
    LoveStatus = WL_CONNECTED;
    return (int) ES_WIFI_SUCCESS; 
}

int WIFI_LOVE::initUDPSockets(void)
{
    if (0 == UDP.begin(commandPort)){
        return (int) ES_WIFI_FAIL_UDP_SOCKET; 
    }

    return (int) ES_WIFI_SUCCESS; 
}

int WIFI_LOVE::fmsgTest_s(void)
{
    UDP.beginPacket(hostIP, sensorPort); 
    UDP.write(TestBuffer);
    UDP.endPacket();
    
    return (int) ES_WIFI_SUCCESS; 
}

int WIFI_LOVE::fmsgBroadcast_s(const uint8_t * buffer, size_t size)
{
    uint8_t res = ES_WIFI_ERROR; 
    if(0 == UDP.beginPacket(broadcastIP, configPort))
    {
        LOG(logWARNING, "Can not connect to the supplied IP or PORT");     
        return  res; 
    } 

    if(size != UDP.write(buffer, size)){
        Serial.println("Size of the UDP Package to big! Truncated overlapping data"); 
    }
    UDP.endPacket();
    return (int) ES_WIFI_SUCCESS; 
}

int WIFI_LOVE::fmsgLogging_s(const uint8_t * buffer, size_t size)
{
    if(UDP.beginPacket(hostIP, logginPort) == 0){
      LOG(logWARNING, "Can not connect to the supplied IP or PORT");
    }
    if(size != UDP.write(buffer, size)){
        Serial.println("Size of the UDP Package to big! Truncated overlapping data"); 
    }
    UDP.endPacket();
    return (int) ES_WIFI_SUCCESS; 
}

int WIFI_LOVE::fmsgSensorDataT_s(const uint8_t * buffer, size_t size)
{
    LOG_d(logVERBOSE, "Send UDP Packet with Timestamp, size: ", size + timestampSize); 
    if(UDP.beginPacket(hostIP, sensorPort) == 0){
      LOG(logWARNING, "Can not connect to the supplied IP or PORT");
    }
    unsigned long t = millis(); 
    uint8_t *addr = (uint8_t*) &t;
    for(int i =  0; i < timestampSize; i++){
        UDP.write(addr[i]);
    }

    if(size != UDP.write(buffer, size)){
        Serial.println("Size of the UDP Package to big! Truncated overlapping data"); 
    }

    UDP.endPacket();
    return (int) ES_WIFI_SUCCESS; 
}

int WIFI_LOVE::fmsgSensorData_s(const uint8_t * buffer, size_t size)
{
    printIP(hostIP);
    UDP.beginPacket(hostIP, sensorPort); 
    if(size != UDP.write(buffer, size)){
        LOG(logERROR,"Size of the UDP Package to big! Truncated overlapping data"); 
    }
    UDP.endPacket();
    return (int) ES_WIFI_SUCCESS; 
}

int WIFI_LOVE::fmsgImuData_s(const uint8_t * buffer, size_t size)
{
    UDP.beginPacket(hostIP, imuPort); 
    if(size != UDP.write(buffer, size)){
        LOG(logERROR,"Size of the UDP Package to big! Truncated overlapping data"); 
    }
    UDP.endPacket();
    return (int) ES_WIFI_SUCCESS; 
}

int WIFI_LOVE::getConnectionStatus(void)
{
    int dV = LoveStatus; 
    return dV; 
}

bool WIFI_LOVE::receiveCommand(){
  int packetSize = UDP.parsePacket(); 
  if(packetSize > 0 )
  {  
//      if(packetSize > 1){
      unsigned char buffer[255]; 
      size_t len = UDP.read(buffer , sizeof buffer); 
      if(len > 0 && len < 60) 
      {
          buffer[len] = '\0'; 
          if(protoLove.decode_command_Proto(buffer, len)){
             command = protoLove.commandObjMsg.command;
          }
      }   
//      }else if(packetSize==1){
//         LOG(logINFO, "triggered!"); 
//         digitalWrite(TRIGGER_PIN, HIGH);
//         delay(1);
//         digitalWrite(TRIGGER_PIN, LOW);
//      }
  }
}

void WIFI_LOVE::checkHostConfig(){
    // wait for a config message
    bool receivedConfig = false;
    while(!receivedConfig)
    {
        if(protoLove.encode_trackedObjConfig( WiFi.localIP(), commandPort, buffer, msg_len)){
            if(fmsgBroadcast_s(buffer, msg_len)){
                LOG(logINFO, "trackedObjectConfig protobuffer successfully sent via UDP Socket"); 
            }else{
                LOG(logERROR, "Sending failed!"); 
            }
        } 
        int packetSize = UDP.parsePacket(); 
        LOG_d(logINFO, "check if command rcvd ", packetSize); 
        if(packetSize > 0 )
        { 
            unsigned char buffer[255]; 
            LOG(logINFO, "rcvd command from host..."); 
            size_t len = UDP.read(buffer , sizeof buffer); 
            if(len > 0 && len < 60) 
            {
                buffer[len] = '\0'; 
                protoLove.decode_config_Proto(buffer, len); 
    
                hostIP = protoLove.configObjMsg.ip;
                logginPort = protoLove.configObjMsg.logging_port; 
                sensorPort = protoLove.configObjMsg.sensor_port;
                imuPort = protoLove.configObjMsg.imu_port;
    
                printIP(hostIP);
                LOG_d(logINFO, "Logging Port Target PC :  ", protoLove.configObjMsg.logging_port); 
                LOG_d(logINFO, "Sensor Port Target PC  :  ", protoLove.configObjMsg.sensor_port); 
                LOG_d(logINFO, "Imu Port Target PC  :  ", protoLove.configObjMsg.imu_port); 
                receivedConfig = true;
            }   
        }
        delay(1000); 
    }    
    if(protoLove.encode_loggingObject( "connection established", buffer, msg_len)){
        if(fmsgLogging_s(buffer, msg_len)){
          LOG(logINFO, "logging sent"); 
        }else{
          LOG(logERROR, "Sending failed!"); 
        }
    } 
}

void WIFI_LOVE::printIP(uint32_t ip){
    Serial.printf("IP: %d.%d.%d.%d\n", (ip>>0)&0xff, (ip>>8)&0xff, (ip>>16)&0xff, (ip>>24)&0xff);
}

bool WIFI_LOVE::sendImuData(Quaternion &q, VectorInt16 &acc, VectorFloat &gravity){
    bool status = protoLove.encode_imuObjConfig(q,acc,gravity,buffer,msg_len);
    fmsgImuData_s(buffer,msg_len);
    return status;
}

