#pragma once
#include <string>
#include <stdint.h>
#include <functional>
using boolean=bool;using TaskHandle_t=void*;
#define F(x) x
#define HEX 16
#define portTICK_PERIOD_MS 1
class String:public std::string {
public:
 using std::string::string;
 String(uint8_t n,int):std::string(std::to_string(n)){}
 bool equals(const String &other) const{return *this==other;}
};
inline void vTaskDelay(unsigned){}
inline unsigned long millis(){return 0;}
inline void xTaskCreatePinnedToCore(void(*)(void*),const char*,unsigned,void*,int,TaskHandle_t*,int){}
