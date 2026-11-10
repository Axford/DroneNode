#ifndef CLOUD_TELEMETRY_HTTP_H
#define CLOUD_TELEMETRY_HTTP_H
#include "TelemetryReport.h"
#include "CloudCommandBatch.h"
#include "FirmwareMeasurements.h"
#include "CloudTelemetryConfig.h"
#include <stdio.h>
#include <stdlib.h>
namespace telemetry {
// Incremental body generator: one base64 group across packet boundaries, no report String.
class PackedBody {
  const HttpSettings &config; const Cycle &cycle;
  size_t first,count,stage=0,position=0,entry=0,byte=0,rawSize=0,consumed=0,qpos=4;
  char idText[4],quartet[4]; bool escape=false;
  const char* part(size_t n) const {
    switch(n) {case 0:return "{\"domain\":\"";case 1:return config.domain;case 2:return "\",\"id\":";case 3:return idText;
      case 4:return ",\"apiKey\":\"";case 5:return config.apiKey;case 6:return "\",\"packets\":\"";default:return "\"}";}
  }
  uint8_t rawNext() {
    const Packet &p=cycle.entries[first+entry].packet;
    uint8_t result=p.bytes[byte++];++consumed;
    if(byte==p.size) {++entry;byte=0;} return result;
  }
  static size_t escapedSize(const char* p) {size_t n=0;for(;*p;++p) n+=(*p=='"'||*p=='\\')?2:1;return n;}
  int next() {
    while(stage<9) {
      if(stage==7) {
        if(qpos==4) {
          if(consumed==rawSize) {++stage;position=0;continue;}
          static const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
          const size_t left=rawSize-consumed;
          uint32_t value=uint32_t(rawNext())<<16;
          if(left>1) value|=uint32_t(rawNext())<<8;
          if(left>2) value|=rawNext();
          quartet[0]=alphabet[(value>>18)&63];quartet[1]=alphabet[(value>>12)&63];
          quartet[2]=left>1?alphabet[(value>>6)&63]:'=';quartet[3]=left>2?alphabet[value&63]:'=';qpos=0;
        }
        return quartet[qpos++];
      }
      const char c=part(stage)[position];
      if(!c) {++stage;position=0;continue;}
      if((stage==1||stage==5)&&(c=='"'||c=='\\')&&!escape) {escape=true;return '\\';}
      escape=false;++position;return static_cast<unsigned char>(c);
    }
    return -1;
  }
public:
  PackedBody(const HttpSettings &cfg,const Cycle &view,size_t offset,size_t length,uint8_t id):config(cfg),cycle(view),first(offset),count(length) {
    snprintf(idText,sizeof idText,"%u",id);
    for(size_t i=0;i<count;++i) rawSize+=cycle.entries[first+i].packet.size;
  }
  size_t size() const {
    size_t total=4*((rawSize+2)/3);
    for(size_t n=0;n<9;++n) if(n!=7) total+=(n==1||n==5)?escapedSize(part(n)):strlen(part(n));
    return total;
  }
  size_t read(uint8_t* output,size_t capacity) {size_t n=0;for(;n<capacity;++n) {int c=next();if(c<0) break;output[n]=c;}return n;}
};
class HttpIo {
public:
  virtual ~HttpIo() {}
  virtual uint32_t now()=0;
  virtual uint32_t nowMicros() {return now()*1000;}
  virtual void observe(Phase,bool,uint32_t) {}
  virtual bool running()=0;
  virtual bool connect(const char* host,uint16_t port)=0;
  virtual size_t write(const uint8_t* bytes,size_t size)=0;
  virtual int read()=0; // Nonblocking; -1 means no byte currently available.
  virtual bool connected()=0;
  virtual void wait()=0;
  virtual void close()=0;
};
class PhaseScope {
  HttpIo &io;Phase phase;uint32_t start;
public:
  PhaseScope(HttpIo &client,Phase value):io(client),phase(value),start(0) {io.observe(phase,false,0);start=io.nowMicros();}
  ~PhaseScope() {io.observe(phase,true,uint32_t(io.nowMicros()-start));}
};
struct Admission { bool attempted=false,complete=false; int status=0,admitted=-1; };
class PackedHttp {
  HttpIo &io;uint32_t started=0;size_t headerBytes=0;
  bool live() {return io.running() && uint32_t(io.now()-started)<45000;}
  bool send(const uint8_t* p,size_t n) {
    PhaseScope measurement(io,Phase::Write);
    while(n&&live()) {size_t done=io.write(p,n);if(!done||done>n) return false;p+=done;n-=done;}
    return !n&&live();
  }
  int take() {
    uint32_t last=io.now();
    while(live()) {
      int c=io.read();if(c>=0) return c;
      if(!io.connected()||uint32_t(io.now()-last)>=5000) return -1;
      io.wait();
    }
    return -1;
  }
  bool line(char* output,size_t cap,bool header=true) {
    size_t n=0;
    while(live()) {
      int c=take();if(c<0) return false;
      if(header&&++headerBytes>8192) return false;
      if(c=='\n') {if(!n||output[n-1]!='\r') return false;--n;output[n]=0;return true;}
      if(!c||n+1>=cap) return false;
      output[n++]=char(c);
    }
    return false;
  }
  static bool equal(const char* a,const char* b) {
    while(*a&&*b) {char x=*a++,y=*b++;if(x>='A'&&x<='Z') x+=32;if(y>='A'&&y<='Z') y+=32;if(x!=y) return false;}return !*a&&!*b;
  }
  static bool number(const char* text,size_t &value,unsigned base=10) {
    if(!*text) return false;value=0;
    for(;*text;++text) {unsigned n=unsigned(*text-'0');if(base==16&&*text>='a'&&*text<='f') n=*text-'a'+10;
      if(base==16&&*text>='A'&&*text<='F') n=*text-'A'+10;
      if(n>=base||value>8192/base) return false;value=value*base+n;if(value>8192) return false;}
    return true;
  }
  static void space(const char* &p) {while(*p==' '||*p=='\r'||*p=='\n'||*p=='\t') ++p;}
  static bool summary(const char* p,size_t expected,int &admitted) {
    unsigned seen=0;size_t received=0,count=0;bool complete=false;
    space(p);if(*p++!='{') return false;
    while(true) {
      space(p);if(*p++!='"') return false;
      const char* key=p;while(*p&&*p!='"') ++p;if(!*p) return false;
      size_t length=p-key;++p;space(p);if(*p++!=':') return false;space(p);
      unsigned bit=0;
      if(length==8&&!memcmp(key,"received",8)) bit=1;
      else if(length==8&&!memcmp(key,"admitted",8)) bit=2;
      else if(length==8&&!memcmp(key,"complete",8)) bit=4;
      else if(length==4&&!memcmp(key,"code",4)) bit=8;
      else return false;
      if(seen&bit) return false;seen|=bit;
      if(bit<=2) {
        char digits[4];size_t n=0;while(*p>='0'&&*p<='9'&&n<3) digits[n++]=*p++;
        digits[n]=0;size_t value;if((n>1&&digits[0]=='0')||!number(digits,value)||value>32) return false;
        if(bit==1) received=value;else count=value;
      } else if(bit==4) {
        if(!strncmp(p,"true",4)) {complete=true;p+=4;}
        else if(!strncmp(p,"false",5)) p+=5;else return false;
      } else {
        const char code[]="\"admission-incomplete\"";
        if(strncmp(p,code,sizeof(code)-1)) return false;p+=sizeof(code)-1;
      }
      space(p);if(*p=='}') {++p;break;}if(*p++!=',') return false;
    }
    space(p);if(*p||!(seen&1)||!(seen&2)||!(seen&4)||received!=expected||count>received) return false;
    admitted=int(count);return complete&&count==expected&&!(seen&8);
  }
  bool readResponse(char* response,size_t capacity,int& status,bool& closeConnection) {
    char input[1025];
    if(!line(input,sizeof input)) return false;
    if(strncmp(input,"HTTP/1.1 ",9)&&strncmp(input,"HTTP/1.0 ",9)) return false;
    if(strlen(input)<12||(input[12]&&input[12]!=' ')||input[9]<'1'||input[9]>'5'||input[10]<'0'||input[10]>'9'||input[11]<'0'||input[11]>'9') return false;
    status=(input[9]-'0')*100+(input[10]-'0')*10+input[11]-'0';
    closeConnection=input[7]=='0'; bool chunked=false,hasLength=false;size_t bodyLength=0;
    while(true) {
      if(!line(input,sizeof input)) return false;if(!input[0]) break;
      char* colon=strchr(input,':');if(!colon) return false;*colon++=0;while(*colon==' '||*colon=='\t') ++colon;
      char* end=colon+strlen(colon);while(end>colon&&(end[-1]==' '||end[-1]=='\t')) *--end=0;
      if(equal(input,"Content-Length")) {if(hasLength||!number(colon,bodyLength)||bodyLength>=capacity) return false;hasLength=true;}
      if(equal(input,"Transfer-Encoding")) {if(chunked||!equal(colon,"chunked")) return false;chunked=true;}
      if(equal(input,"Connection")&&equal(colon,"close")) closeConnection=true;
      if(equal(input,"Content-Encoding")&&!equal(colon,"identity")) return false;
    }
    if((hasLength&&chunked)||(!hasLength&&!chunked)) return false;
    size_t used=0;
    do {
      size_t part=bodyLength;
      if(chunked) {
        if(!line(input,sizeof input)) return false;
        if(!number(input,part,16)||part>=capacity-used) return false;
        if(!part) { // bounded trailers
          do {if(!line(input,sizeof input)) return false;} while(input[0]);
          break;
        }
      }
      if(part>=capacity-used) return false;
      for(size_t n=0;n<part;++n) {int c=take();if(c<=0) return false;response[used++]=char(c);}
      if(chunked&&(take()!='\r'||take()!='\n')) return false;
    } while(chunked);
    response[used]=0;
    return true;
  }
  static void escape(const char* input,char* output) {
    for(;*input;++input) {
      if(*input=='"'||*input=='\\') *output++='\\';
      *output++=*input;
    }
    *output=0;
  }
public:
  explicit PackedHttp(HttpIo &client):io(client) {}
  Admission poll(const HttpSettings &cfg,uint8_t id,size_t limit,CommandBatch& batch) {
    Admission result;
    batch.count=0;started=io.now();headerBytes=0;
    if(!id||id>=255||!limit||limit>BatchLimit||!live()) return result;
    result.attempted=true;
    PhaseScope request(io,Phase::Request);
    {
      PhaseScope connect(io,Phase::Connect);
      if(!io.connect(cfg.host,cfg.port)||!live()) {io.close();return result;}
    }
    char domain[401],key[1025],body[1536],header[512];
    escape(cfg.domain,domain);escape(cfg.apiKey,key);
    int length=snprintf(body,sizeof body,"{\"domain\":\"%s\",\"id\":%u,\"apiKey\":\"%s\",\"limit\":%u}",domain,id,key,unsigned(limit));
    if(length<0||size_t(length)>=sizeof body) {io.close();return result;}
    int n=snprintf(header,sizeof header,"POST /commands/poll HTTP/1.1\r\nHost: %s:%u\r\nContent-Type: application/json\r\nContent-Length: %u\r\nConnection: close\r\n\r\n",cfg.host,cfg.port,unsigned(length));
    if(n<0||size_t(n)>=sizeof header||!send(reinterpret_cast<const uint8_t*>(header),n)||
       !send(reinterpret_cast<const uint8_t*>(body),length)) {io.close();return result;}
    PhaseScope responsePhase(io,Phase::Response);
    char response[2049];bool closeConnection=false;
    result.complete=readResponse(response,sizeof response,result.status,closeConnection)&&result.status==200&&live()&&batch.parse(response,id,limit);
    io.close();
    return result;
  }
  Admission sendBatch(const HttpSettings &cfg,const Cycle &cycle,size_t first,size_t count,uint8_t id) {
    Admission result;started=io.now();headerBytes=0;
    if(!count||count>32||first+count>cycle.count||!live()) return result;
    PhaseScope requestMeasurement(io,Phase::Request);
    PackedBody body(cfg,cycle,first,count,id);
    result.attempted=true;
    bool connected;
    {
      PhaseScope connectMeasurement(io,Phase::Connect);
      connected=io.connect(cfg.host,cfg.port);
    }
    if(!connected||!live()) {io.close();return result;}
    char header[512];int length=snprintf(header,sizeof header,"POST /addPackets HTTP/1.1\r\nHost: %s:%u\r\nContent-Type: application/json\r\nContent-Length: %u\r\nConnection: keep-alive\r\n\r\n",cfg.host,cfg.port,unsigned(body.size()));
    if(length<0||size_t(length)>=sizeof header||!send(reinterpret_cast<const uint8_t*>(header),length)) {io.close();return result;}
    uint8_t buffer[256];size_t bytes;
    while(true) {
      {PhaseScope encodingMeasurement(io,Phase::Encode);bytes=body.read(buffer,sizeof buffer);}
      if(!bytes) break;
      if(!send(buffer,bytes)) {io.close();return result;}
    }
    PhaseScope responseMeasurement(io,Phase::Response);
    char response[1025];bool closeConnection=false;
    if(!readResponse(response,sizeof response,result.status,closeConnection)) {io.close();return result;}
    const bool full=summary(response,count,result.admitted);
    result.complete=result.status==202&&full&&live();
    if(!result.complete||closeConnection) io.close();
    return result;
  }
};
}
#endif
