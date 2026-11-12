#pragma once
#include <stdint.h>
#include <stddef.h>
#include <vector>
#include <cstring>
typedef bool boolean;
#define PROGMEM
#define F(x) x
#define FPSTR(x) x
#define SERIAL_8N1 0
inline unsigned long millis(){return 1;}
class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t byte) {return write(&byte,1);}
  virtual size_t write(const uint8_t*,size_t)=0;
};
class Stream: public Print {
 public:
  std::vector<uint8_t> input,output; size_t pos=0;
  virtual ~Stream() {}
  int available(){return input.size()-pos;}
  int read(){return available()?input[pos++]:-1;}
  size_t write(const uint8_t* b,size_t n){output.insert(output.end(),b,b+n);return n;}
  int availableForWrite(){return 128;}
  void flush(){} void begin(uint32_t,int=0,int=0,int=0){} void updateBaudRate(uint32_t){}
};
extern Stream Serial,Serial1,Serial2;
struct Logger { int getLevel(){return 6;} void setLevel(int){} template<class... T> void errorln(T...){} template<class... T> void noticeln(T...){} };
extern Logger Log;
