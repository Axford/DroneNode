#ifndef CLOUD_TELEMETRY_CONFIG_H
#define CLOUD_TELEMETRY_CONFIG_H
#include <stdint.h>
#include "ConfigurationLine.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>
namespace telemetry {
using Setting=configuration::Setting;
struct Selection {
  char node[20]={}, channel[20]={}, param[20]={};
  uint8_t resolvedNode=0, resolvedChannel=0, resolvedParam=0;
  bool resolved=false, subscribed=false;
};
inline bool decimal(const char* text,unsigned max,unsigned &value) {
  if(!text || !*text) return false;
  value=0;
  for(const char* p=text;*p;++p) {
    if(*p<'0'||*p>'9'||value>max/10) return false;
    value=value*10+unsigned(*p-'0'); if(value>max) return false;
  }
  return value>0;
}
inline bool printable(const char* value,size_t max) {
  if(!value || !*value || strlen(value)>max) return false;
  for(const unsigned char* p=reinterpret_cast<const unsigned char*>(value);*p;++p) if(*p<32 || *p>126) return false;
  return true;
}
struct HttpSettings {
  char endpoint[257]={},host[254]={},domain[201]={},apiKey[513]={};
  uint16_t port=443;
};
class Config:public HttpSettings {
  size_t selectionBytes=0;
  bool invalid=false;
  Setting fail(const char* reason) { invalid=true; error=reason; return Setting::Invalid; }
  static bool token(char* dst,const char* begin,size_t length) {
    while(length && (*begin==' '||*begin=='\t')) {++begin;--length;}
    while(length && (begin[length-1]==' '||begin[length-1]=='\t')) --length;
    if(!length || length>19) return false;
    for(size_t i=0;i<length;++i) {
      char c=begin[i];
      if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-'||c=='@'||c=='*')) return false;
    }
    memcpy(dst,begin,length);dst[length]=0;return true;
  }
  bool selection(const char* begin,size_t length,Selection &entry) {
    const char* end=begin+length; const char* arrow=nullptr;const char* dot=nullptr;
    for(const char* p=begin;p<end;++p) { if(*p=='>') { if(arrow) return false; arrow=p; } if(*p=='.') {if(dot) return false;dot=p;} }
    if(!arrow||!dot||arrow>=dot) return false;
    if(!token(entry.node,begin,arrow-begin)||!token(entry.channel,arrow+1,dot-arrow-1)||!token(entry.param,dot+1,end-dot-1)) return false;
    unsigned value;
    const bool local=strcmp(entry.node,"@")==0;
    if(!local && (strchr(entry.node,'@')||strchr(entry.node,'*'))) return false;
    if(entry.node[0]>='0'&&entry.node[0]<='9' && !decimal(entry.node,254,value)) return false;
    if(strchr(entry.channel,'@')||strchr(entry.channel,'*')) return false;
    const bool numericChannel=decimal(entry.channel,254,value);
    if((entry.channel[0]>='0'&&entry.channel[0]<='9')&&!numericChannel) return false;
    const bool wildcard=strcmp(entry.param,"*")==0;
    const bool numericParam=decimal(entry.param,63,value);
    if(!wildcard && (strchr(entry.param,'@')||strchr(entry.param,'*'))) return false;
    if(!wildcard && entry.param[0]>='0'&&entry.param[0]<='9'&&!numericParam) return false;
    return local || (numericChannel && (wildcard||numericParam));
  }
  bool parseEndpoint(const char* value) {
    if(!printable(value,256)||strncmp(value,"https://",8)!=0) return false;
    const char* authority=value+8; const char* colon=strchr(authority,':');
    const size_t length=colon ? size_t(colon-authority):strlen(authority);
    if(!length||length>253) return false;
    for(size_t i=0;i<length;++i) {
      char c=authority[i];if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='.'||c=='-')) return false;
    }
    unsigned candidate=443;
    if(colon&&!decimal(colon+1,65535,candidate)) return false;
    memcpy(host,authority,length);host[length]=0;port=candidate;strcpy(endpoint,value);return true;
  }
public:
  Selection selections[64];size_t count=0;
  const char* error=nullptr;
  Setting apply(const char* name,const char* value) {
    if(!strcmp(name,"endpoint")) return parseEndpoint(value) ? Setting::Handled:fail("invalid endpoint");
    if(!strcmp(name,"domain")) {
      if(!printable(value,200)||strchr(value,'/')||(strlen(value)>=4&&!strncmp(value,"__",2)&&!strcmp(value+strlen(value)-2,"__"))||!strcmp(value,".")||!strcmp(value,"..")) return fail("invalid domain");
      strcpy(domain,value);return Setting::Handled;
    }
    if(!strcmp(name,"apiKey")) {
      if(!printable(value,512)) return fail("invalid apiKey");
      strcpy(apiKey,value);return Setting::Handled;
    }
    if(strcmp(name,"subscriptions")) return Setting::Unhandled;
    if(!value || !*value || strlen(value)>768) return fail("invalid subscriptions");
    const char* begin=value;
    do {
      const char* end=strchr(begin,','); if(!end) end=begin+strlen(begin);
      Selection entry;
      if(!selection(begin,end-begin,entry)) return fail("malformed subscription");
      bool duplicate=false;
      for(size_t i=0;i<count;++i) if(!strcmp(entry.node,selections[i].node)&&!strcmp(entry.channel,selections[i].channel)&&!strcmp(entry.param,selections[i].param)) duplicate=true;
      if(!duplicate) {
        const size_t bytes=strlen(entry.node)+strlen(entry.channel)+strlen(entry.param)+3;
        if(count==64||selectionBytes+bytes>4096) return fail("subscription capacity");
        selections[count++]=entry;selectionBytes+=bytes;
      }
      if(!*end) break;
      begin=end+1;
    } while(true);
    return Setting::Handled;
  }
  bool complete() const { return !invalid && endpoint[0]&&domain[0]&&apiKey[0]&&count; }
  void invalidate(const char* reason) { fail(reason); }
};
static_assert(sizeof(Config)<=8192,"bounded CloudTelemetry configuration");
}
#endif
