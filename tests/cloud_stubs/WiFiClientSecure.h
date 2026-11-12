#pragma once
#include "DroneModule.h"
extern std::string requestBytes,responseBytes;
extern std::function<void()> duringWrite;
struct WiFiClientSecure {
 bool open=false;size_t pos=0;
 void setInsecure(){}void setHandshakeTimeout(unsigned){}void setTimeout(unsigned){}
 bool connected(){return open;}bool connect(const char*,unsigned,int){open=true;pos=0;return true;}
 unsigned available(){return responseBytes.size()-pos;}int read(){return pos<responseBytes.size()?static_cast<unsigned char>(responseBytes[pos++]):-1;}
 size_t write(const uint8_t* bytes,size_t n){requestBytes.append(reinterpret_cast<const char*>(bytes),n);if(duringWrite){auto callback=duringWrite;duringWrite=nullptr;callback();}return n;}
 void stop(){open=false;}
};
