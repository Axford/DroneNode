#pragma once
#include "Arduino.h"
#define WL_CONNECTED 3
#define WIFI_STA 1
#define WIFI_OFF 0
#define WIFI_POWER_MINUS_1dBm 1
#define WIFI_SCAN_RUNNING -1
#define WIFI_SCAN_FAILED -2
#define ARDUINO_EVENT_WIFI_STA_GOT_IP 7
using WiFiEvent_t=int;using WiFiEventInfo_t=int;
struct WiFiClass {
 std::function<void(int,int)> callback;unsigned registrations=0;
 void onEvent(std::function<void(int,int)> c,int){callback=c;++registrations;}
 void mode(int){}void setTxPower(int){}void macAddress(uint8_t* m){for(int i=0;i<6;++i)m[i]=0;}
 void disconnect(){}void begin(const char*,const char*){}void setSleep(bool){}
 int status(){return WL_CONNECTED;}int localIP(){return 0;}
 void scanNetworks(bool,bool,bool,int,int){}int16_t scanComplete(){return -1;}String SSID(uint8_t){return "fixture";}
};extern WiFiClass WiFi;
