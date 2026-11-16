#ifndef DNS_OVERRIDE_H
#define DNS_OVERRIDE_H
#include <stdint.h>
// Configuration-only IPv4 resolver. Default policy stays with WiFiManager.
struct DnsOverride {
  bool configured=false;
  uint8_t bytes[4]={};
  bool parse(const char* text) {
    configured=false;
    if(!text)return false;
    const char* p=text;
    for(unsigned i=0;i<4;++i) {
      unsigned value=0,digits=0;const char* start=p;
      while(*p>='0'&&*p<='9') {value=value*10+unsigned(*p++-'0');if(++digits>3||value>255)return false;}
      if(!digits||(digits>1&&*start=='0'))return false;
      bytes[i]=uint8_t(value);
      if(i<3) {if(*p++!='.')return false;}else if(*p)return false;
    }
    if(!bytes[0]||bytes[0]>=224||(bytes[0]==127))return false;
    configured=true;return true;
  }
};
#endif
