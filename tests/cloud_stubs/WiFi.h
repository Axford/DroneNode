#pragma once
#define WL_CONNECTED 3
struct WiFiClass{int state=3;int status(){return state;}};extern WiFiClass WiFi;
