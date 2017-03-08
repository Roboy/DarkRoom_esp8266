

#ifndef WIRELESSLOVE_H
#define WIRELESSLOVE_H

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiAP.h>
#include <WiFiServer.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiType.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiSTA.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include "logging.h"
#include "protoLighthouse.h"

class WIFI_LOVE{
public:
    WIFI_LOVE(const char* ssid, const char* passwd, IPAddress &broadcastIP);
    uint16_t getCmndPort(); 
    uint32_t getLocalIP(); 

    int     printWifiStatus(); 
    int     initWifi(); 
    int     initUDPSockets(); 
    int     fmsgTest_s(); 
    int     fmsgBroadcast_s(const uint8_t * buffer, size_t size); 
    int     fmsgLogging_s(const uint8_t * buffer, size_t size); 
    int     fmsgSensorDataT_s(const uint8_t *buffer, size_t size); 
    int     fmsgSensorData_s(const uint8_t *buffer, size_t size); 
    int     fmsgImuData_s(const uint8_t *buffer, size_t size); 
    bool    receiveCommand();
    bool    receiveConfig(); 
    bool    sendImuData(Quaternion &q, VectorInt16 &acc, VectorFloat &gravity);
    int     getConnectionStatus();
    void    checkHostConfig();
    void    printIP(uint32_t ip);

    int      LoveStatus = WL_IDLE_STATUS;
    char     ssid[100]; 
    char     pass[100]; 
    bool     timeout = false; 

    uint16_t  sensorPort   = 0; // will be received from host 
    uint16_t  commandPort  = 4210; 
    uint16_t  logginPort   = 0;  // will be received from host 
    uint16_t  configPort   = 8001;  
    uint16_t  imuPort   = 0; // will be received from host 

    IPAddress hostIP;;
    IPAddress broadcastIP; 

    WiFiUDP  UDP;
    PROTO_LOVE protoLove;
    uint32_t command;

    size_t msg_len;
    pb_byte_t buffer[512] = {0};
};


#endif
