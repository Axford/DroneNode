#include "TelemetryReport.h"
#include "CloudTelemetryConfig.h"
#include "ConfigurationLine.h"
#include "CloudTelemetryHttp.h"
#include <cassert>
#include <string>
#include <vector>
struct FakeHttp:telemetry::HttpIo {
 std::string sent,response;size_t position=0,limit=7;uint32_t time=0,connectDelay=0;bool open=true,active=true,connects=true,cancelOnWrite=false;
 uint32_t now()override{return time;}bool running()override{return active;}
 bool connect(const char*,uint16_t)override{time+=connectDelay;return connects;}
 size_t write(const uint8_t* p,size_t n)override{if(cancelOnWrite)active=false;if(n>limit)n=limit;sent.append(reinterpret_cast<const char*>(p),n);return n;}
 int read()override{return position<response.size()?static_cast<unsigned char>(response[position++]):-1;}
 bool connected()override{return open;}void wait()override{time+=100;}void close()override{open=false;}
};
static telemetry::Config settings() {
 telemetry::Config c;c.apply("endpoint","https://example.test");c.apply("domain","fixture");c.apply("apiKey","secret");c.apply("subscriptions","@>1.*");return c;
}
static telemetry::Cycle literalCycle() {
 telemetry::Cycle c;c.count=2;
 const uint8_t first[]={0,11,4,8,0,9}; // AAsECAAJ
 const uint8_t second[]={12,12,5,10,0x42,'A','B','C'}; // DAwFCkJBQkM=
 memcpy(c.entries[0].packet.bytes,first,sizeof first);c.entries[0].packet.size=sizeof first;
 memcpy(c.entries[1].packet.bytes,second,sizeof second);c.entries[1].packet.size=sizeof second;
 return c;
}
static void configurationTests() {
 configuration::Line line;
 assert(line.parse("  apiKey = \"a\\\"b\\\\c\" ; comment",true));assert(!strcmp(line.value,"a\"b\\c"));
 assert(line.parse("[ CloudTelemetry = 20 ]",false)&&line.section);assert(!strcmp(line.name,"CloudTelemetry")&&!strcmp(line.value,"20"));
 assert(line.parse(" publish = status, name ; comment",false)&&!strcmp(line.value,"status,name"));
 assert(line.parse("name=\"with spaces\"",false)&&!strcmp(line.value,"with spaces"));
 assert(line.parse("  ; empty",false)&&line.empty);
 assert(!line.parse("key=\"unterminated",true));assert(!line.parse("key=\"bad\\q\"",true));
 std::string longKey(512,'x');assert(line.parse(("apiKey="+longKey).c_str(),true)&&strlen(line.value)==512);
 assert(!line.parse(("apiKey="+std::string(769,'x')).c_str(),true));
 auto c=settings();assert(c.complete());
 assert(c.apply("subscriptions","@>GPS.location, Hull>4.8")==telemetry::Setting::Handled);
 assert(c.apply("subscriptions","12>5.*, @>GPS.location")==telemetry::Setting::Handled);assert(c.count==4);
 assert(c.apply("apiKey",longKey.c_str())==telemetry::Setting::Handled);
 assert(c.apply("subscriptions","Hull>GPS.location")==telemetry::Setting::Invalid&&!c.complete());
 for(const char* bad:{"http://example.test","https://name/path","https://user@name","https://name:0","https://name:65536","https://name?query","https://name#fragment"}) {
   telemetry::Config invalid;assert(invalid.apply("endpoint",bad)==telemetry::Setting::Invalid);
 }
 for(const char* bad:{"0>1.1","255>1.1","12>0.1","12>1.0","12>1.64","12>1.*x","@>1.1,","@>1.1garbage","12>1>2.1"}) {
   auto invalid=settings();assert(invalid.apply("subscriptions",bad)==telemetry::Setting::Invalid);
 }
 assert(!telemetry::Report::validInterval(0)&&!telemetry::Report::validInterval(999));
 assert(telemetry::Report::validInterval(86400000)&&!telemetry::Report::validInterval(86400001));
}
static void policyTests() {
 telemetry::Report report;telemetry::Cycle cycle;
 uint8_t bytes[]={0,11,4,8,0,7};
 report.start(100,10000);assert(report.capture(bytes,sizeof bytes));bytes[5]=9;assert(report.capture(bytes,sizeof bytes));
 assert(!report.begin(10099,true,cycle));assert(report.begin(10100,true,cycle));assert(cycle.count==1&&cycle.entries[0].packet.bytes[5]==9);
 report.capture(bytes,sizeof bytes);report.finish(cycle,1);assert(report.begin(20100,true,cycle)&&cycle.count==1);report.finish(cycle,1);
 assert(!report.begin(30100,true,cycle));
 // Offline ticks retain latest; reconnect is not a reporting tick.
 report.capture(bytes,sizeof bytes);assert(!report.begin(40100,false,cycle));assert(!report.begin(40101,true,cycle));
 assert(report.begin(50100,true,cycle));report.finish(cycle,1);
 // Independent NAME/value and same numeric channel/param on different nodes.
 bytes[4]=0x50;bytes[5]='X';report.capture(bytes,sizeof bytes);bytes[4]=0;report.capture(bytes,sizeof bytes);
 bytes[1]=12;report.capture(bytes,sizeof bytes);assert(report.begin(60100,true,cycle)&&cycle.count==3);
 report.finish(cycle,1);assert(report.begin(70100,true,cycle)&&cycle.count==2);
 // Incomplete cycle retains remainder; new arrivals supersede frozen originals.
 auto old=cycle;bytes[5]=33;report.capture(bytes,sizeof bytes);report.finish(old,1);
 assert(report.begin(80100,true,cycle)&&cycle.count>=1);
 report.stop();report.start(90000,10000);report.finish(old,old.count);assert(!report.begin(100000,true,cycle));
 for(unsigned i=1;i<=64;++i){bytes[1]=i;assert(report.capture(bytes,sizeof bytes));}
 bytes[1]=65;assert(!report.capture(bytes,sizeof bytes)&&report.overflow==1&&report.tracked()==64);
 bytes[1]=1;assert(report.capture(bytes,sizeof bytes));assert(report.begin(110000,true,cycle)&&cycle.count==64);
 uint8_t firstNode=cycle.entries[0].packet.bytes[1];report.finish(cycle,32);
 for(unsigned i=1;i<=64;++i){bytes[1]=i;report.capture(bytes,sizeof bytes);}
 assert(report.begin(120000,true,cycle));assert(cycle.entries[0].packet.bytes[1]!=firstNode);
 // unsigned rollover, reconfiguration, and generation fencing.
 report.stop();report.start(0xfffffff0u,1000);bytes[1]=1;report.capture(bytes,sizeof bytes);
 assert(!report.begin(983,true,cycle));assert(report.begin(984,true,cycle));report.finish(cycle,1);
 report.setInterval(990,2000);report.capture(bytes,sizeof bytes);assert(!report.begin(2989,true,cycle));assert(report.begin(2990,true,cycle));
 auto wrong=literalCycle();wrong.entries[0].packet.bytes[4]=0x20;assert(!telemetry::publication(wrong.entries[0].packet.bytes,6));
 for(unsigned type=6;type<8;++type){bytes[4]=type<<4;assert(!report.capture(bytes,sizeof bytes));}
 bytes[4]=0;bytes[0]=99;assert(!report.capture(bytes,sizeof bytes));bytes[0]=0;bytes[1]=255;assert(!report.capture(bytes,sizeof bytes));
}
static telemetry::Admission response(const std::string &wire,FakeHttp &io) {
 io.response=wire;auto c=settings();auto cycle=literalCycle();telemetry::PackedHttp http(io);return http.sendBatch(c,cycle,0,2,11);
}
static std::string framed(const std::string& body,int status=202) {
 return "HTTP/1.1 "+std::to_string(status)+" result\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
}
static void httpTests() {
 const std::string full="{\"received\":2,\"admitted\":2,\"complete\":true}";
 FakeHttp io;auto result=response(framed(full),io);assert(result.complete&&result.admitted==2);
 // Literal independent base64, concatenated across packet boundaries with one final padding.
 const std::string body="{\"domain\":\"fixture\",\"id\":11,\"apiKey\":\"secret\",\"packets\":\"AAsECAAJDAwFCkJBQkM=\"}";
 assert(io.sent.substr(io.sent.find("\r\n\r\n")+4)==body);
 assert(io.sent.find("Content-Length: "+std::to_string(body.size())+"\r\n")!=std::string::npos);
 assert(io.open); // Reusable on a following request; no implicit reconnect/retry.
 FakeHttp chunks;
 auto chunked="HTTP/1.1 202 yes\r\nTransfer-Encoding: chunked\r\n\r\n";
 char size[16];snprintf(size,sizeof size,"%x",unsigned(full.size()));
 assert(response(std::string(chunked)+size+"\r\n"+full+"\r\n0\r\n\r\n",chunks).complete);
 for(const auto& bad:{framed(full,200),std::string("HTTP/1.1 2020 bogus\r\nContent-Length: ")+std::to_string(full.size())+"\r\n\r\n"+full,framed("{\"received\":2,\"admitted\":1,\"complete\":false,\"code\":\"admission-incomplete\"}",503),
   framed("{\"received\":1,\"admitted\":1,\"complete\":true}"),framed(full+"garbage"),
   std::string("HTTP/1.1 302 redirect\r\nContent-Length: 0\r\nLocation: http://other\r\n\r\n"),
   std::string("HTTP/1.1 202 yes\r\nContent-Length: 99999\r\n\r\n"),
   std::string("HTTP/1.1 202 yes\r\nContent-Length: 1\r\nTransfer-Encoding: chunked\r\n\r\n"),
   std::string("HTTP/1.1 202 yes\r\nX-Large: ")+std::string(1100,'a')+"\r\n\r\n"}) {
   FakeHttp broken;assert(!response(bad,broken).complete&&!broken.open);
 }
 FakeHttp timeout;assert(!response("",timeout).complete&&timeout.time>=5000&&timeout.time<=5100);
 FakeHttp dns;dns.connectDelay=31000;assert(response(framed(full),dns).complete); // Longer accepted library wait.
 FakeHttp deadline;deadline.connectDelay=45001;assert(!response(framed(full),deadline).complete&&deadline.sent.empty());
 FakeHttp cancel;cancel.cancelOnWrite=true;assert(!response(framed(full),cancel).complete&&!cancel.open);
 FakeHttp writes;writes.limit=0;assert(!response(framed(full),writes).complete&&!writes.open);
 FakeHttp fail;fail.connects=false;assert(!response(framed(full),fail).complete&&fail.sent.empty());
 // Every truncation must fail; malformed small responses must remain memory-safe.
 for(size_t n=0;n<full.size();++n) {FakeHttp truncated;assert(!response(framed(full.substr(0,n)),truncated).complete);}
 for(const char* small:{"", "{", "{\"", "{\"received\"", "{\"received\":", "{}", "null", "[]"}) {FakeHttp f;assert(!response(framed(small),f).complete);}
 auto cfg=settings();cfg.apply("domain","a\"b\\c");auto cycle=literalCycle();telemetry::PackedBody escaped(cfg,cycle,0,2,11);
 std::string output;uint8_t buffer[3];size_t n;while((n=escaped.read(buffer,sizeof buffer))) output.append(reinterpret_cast<char*>(buffer),n);
 assert(output.size()==escaped.size()&&output.find("a\\\"b\\\\c")!=std::string::npos);
}
int main(){configurationTests();policyTests();httpTests();}
