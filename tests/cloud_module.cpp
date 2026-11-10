#include "droneModules/CloudTelemetryModule.h"
#include "DroneSystem.h"
#include "WiFi.h"
#include <cassert>
#include "actual_save.inc"
uint32_t hostTime=0;Logger Log;ESPClass ESP;WiFiClass WiFi;thread_local Task* currentTask=nullptr;
std::string configText;HostFS LITTLEFS;bool taskAllocationFails=false;std::string requestBytes,responseBytes;std::function<void()> duringWrite;
static DroneLinkMsg message(uint8_t node,uint8_t channel,uint8_t param,uint8_t type=0,uint8_t value=9) {
 DroneLinkMsg msg;msg._msg.node=node;msg._msg.channel=channel;msg._msg.param=param;msg._msg.typed=type;msg._msg.payload.uint8[0]=value;return msg;
}
static void config(CloudTelemetryModule& module,bool poll=false) {
 if(!poll)module._params[6].data.uint32[0]=0;
 module.configure("endpoint","https://example.test");module.configure("domain","fixture");module.configure("apiKey","0123456789012345678901234567890123456789012345678901234567890123");
 module.configure("subscriptions","@>GPS.location");module.configure("subscriptions","Hull>5.*");
}
int main() {
 DroneLinkManager dlm;DroneModuleManager dmm;DroneSystem system{&dlm,&dmm};DroneModule gps(4,&system);dmm.gps=&gps;
 const std::string body="{\"received\":1,\"admitted\":1,\"complete\":true}";
 responseBytes="HTTP/1.1 202 Accepted\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
 {
  CloudTelemetryModule module(20,&system);config(module);module.setup();module.loop();
  assert(dlm.subs.size()==1&&dlm.subs[0].node==11&&dlm.subs[0].param==8);
  auto msg=message(11,4,8);dlm.publishLocal(msg);
  hostTime=9999;module.loop();assert(requestBytes.empty());hostTime=10000;module.loop();assert(!requestBytes.empty());
  module.loop();assert(module._params[1].data.uint32[0]==1&&module._params[1].data.uint32[1]==1);
  requestBytes.clear();hostTime=20000;module.loop();assert(requestBytes.empty());
  dlm.hull=true;hostTime=21000;module.loop();assert(dlm.subs.size()==2&&dlm.subs[1].node==12&&dlm.subs[1].param==0);
  auto peer=message(12,5,10);module.handleLinkMessage(&peer);hostTime=30000;module.loop();assert(requestBytes.find("AAwFCgAJ")!=std::string::npos);module.loop();
  // Disable during transmission, re-enable before old completion: no old counters enter new lifecycle.
  requestBytes.clear();dlm.publishLocal(msg);auto oldRequests=module._params[1].data.uint32[0];
  duringWrite=[&](){module.disable();module.enable();};hostTime=40000;module.loop();module.loop();assert(module._params[1].data.uint32[0]==oldRequests);
  requestBytes.clear();hostTime=50000;module.loop();assert(requestBytes.empty());
  File file;module.saveExtraConfiguration(&file);assert(file.text.find("subscriptions = @>GPS.location")!=std::string::npos&&file.text.find("subscriptions = Hull>5.*")!=std::string::npos);
  assert(file.text.find("0123456789012345678901234567890123456789012345678901234567890123")!=std::string::npos);
  module.disable();module.enable();dlm.publishLocal(msg);WiFi.state=0;hostTime=60000;module.loop();assert(requestBytes.empty());
  WiFi.state=3;hostTime=60001;module.loop();assert(requestBytes.empty());hostTime=70000;module.loop();assert(!requestBytes.empty());module.loop();
  auto change=message(11,20,8,0x23);change._msg.payload.uint32[0]=2000;module.handleLinkMessage(&change);assert(module._params[0].data.uint32[0]==2000);
  change._msg.typed=0x33;change._msg.payload.uint32[0]=9999;module.handleLinkMessage(&change);assert(module._params[0].data.uint32[0]==2000);
  module.doShutdown();assert(!module._enabled);
 }
 {
  CloudTelemetryModule invalid(20,&system);config(invalid);invalid.configure("subscriptions","Hull>GPS.location");invalid.setup();assert(!invalid._enabled&&invalid._error);
 }
 {
  CloudTelemetryModule first(20,&system);config(first);CloudTelemetryModule second(21,&system);config(second);first.setup();second.setup();assert(!first._enabled&&!second._enabled);
 }
 {
  hostTime=80000;requestBytes.clear();CloudTelemetryModule batches(20,&system);config(batches);batches.configure("subscriptions","@>1.*");batches.setup();batches.loop();
  for(unsigned param=1;param<=63;++param) {
    auto value=message(11,1,param),name=message(11,1,param,0x50,'N');dlm.publishLocal(value);dlm.publishLocal(name);
  }
  auto peerValue=message(12,5,1),peerName=message(12,5,1,0x50,'P');batches.handleLinkMessage(&peerValue);batches.handleLinkMessage(&peerName);
  auto frame=[](const std::string& body,unsigned status){return "HTTP/1.1 "+std::to_string(status)+" result\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;};
  const std::string full="{\"received\":32,\"admitted\":32,\"complete\":true}";
  responseBytes=frame(full,202)+frame("{\"received\":32,\"admitted\":31,\"complete\":false,\"code\":\"admission-incomplete\"}",503);
  hostTime=90000;batches.loop();batches.loop();assert(batches._params[1].data.uint32[0]==2&&batches._params[1].data.uint32[1]==63&&batches._params[1].data.uint32[2]==1);
  requestBytes.clear();responseBytes=frame(full,202)+frame(full,202);hostTime=100000;batches.loop();batches.loop();assert(batches._params[1].data.uint32[0]==4&&batches._params[1].data.uint32[1]==127);
  requestBytes.clear();hostTime=110000;batches.loop();assert(requestBytes.empty());
 }
 {
  hostTime=120000;requestBytes.clear();dlm.issued.clear();
  CloudTelemetryModule module(20,&system);config(module,true);module.setup();module.loop();
  const auto frame=[](unsigned status,const std::string& body){
   return "HTTP/1.1 "+std::to_string(status)+" result\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
  };
  responseBytes=frame(200,R"({"success":true,"data":{"count":2,"packets":"CwsUziMAAAAACwsEDGAA"}})");
  hostTime=129999;module.loop();assert(requestBytes.empty());
  hostTime=130000;module.loop();assert(requestBytes.find("POST /commands/poll ")==0&&requestBytes.find("\"limit\":32")!=std::string::npos);
  dlm.fullCloud=true;module.loop();assert(dlm.issued.empty());
  dlm.fullCloud=false;module.loop();assert(dlm.issued.size()==1&&dlm.issued.back().param()==14);
  module.handleLinkMessage(&dlm.issued.back());dlm.pendingCloud=false;
  assert(module._params[6].data.uint32[0]==0);
  module.loop();assert(dlm.issued.size()==2&&dlm.issued.back().type()==7);
  dlm.pendingCloud=false;module.loop();
  assert(dlm.issued.size()==3&&dlm.issued.back().type()==6&&dlm.issued.back().param()==12);
  dlm.pendingCloud=false;module.loop();
  requestBytes.clear();
  responseBytes=frame(202,"{\"received\":1,\"admitted\":1,\"complete\":true}");
  auto reply=message(11,4,12,0x50,'N');dlm.publishLocal(reply);
  ++hostTime;module.loop();
  assert(requestBytes.find("POST /addPackets ")==0&&requestBytes.find("AAsEDFBO")!=std::string::npos);
  module.loop();requestBytes.clear();
  dlm.publishLocal(reply);hostTime=150001;module.loop();
  assert(requestBytes.empty()&&dlm.issued.size()==3);
 }
 {
  hostTime=200000;requestBytes.clear();dlm.issued.clear();
  CloudTelemetryModule module(20,&system);config(module,true);module.setup();module.loop();
  const auto frame=[](unsigned status,const std::string& body){
   return "HTTP/1.1 "+std::to_string(status)+" result\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
  };
  responseBytes=frame(200,R"({"success":true,"data":{"count":1,"packets":"CwsECHAA"}})");
  hostTime=210000;module.loop();module.loop();
  assert(dlm.issued.size()==1&&dlm.issued.back().type()==7);
  dlm.pendingCloud=false;module.loop();
  auto stop=message(11,20,14,0x23);stop._msg.payload.uint32[0]=0;module.handleLinkMessage(&stop);
  auto old=message(11,4,8,0,10),newer=message(11,4,8,0,20);
  auto injected=message(11,4,8,0,99);
  module.handleLinkMessage(&injected);
  dlm.publishLocal(old);dlm.publishLocal(newer);
  responseBytes=frame(202,R"({"received":1,"admitted":1,"complete":true})");
  requestBytes.clear();hostTime=220000;module.loop();
  assert(requestBytes.find("AAsECAAK")!=std::string::npos&&requestBytes.find("AAsECAAU")==std::string::npos);
  module.loop();
  requestBytes.clear();hostTime=230000;module.loop();
  assert(requestBytes.find("AAsECAAU")!=std::string::npos&&requestBytes.find("AAsECAAK")==std::string::npos);
 }
 {
  dlm.address=0;CloudTelemetryModule missingNode(20,&system);config(missingNode);missingNode.setup();assert(!missingNode._enabled&&missingNode._error);dlm.address=11;
 }
 {
  taskAllocationFails=true;CloudTelemetryModule unavailable(20,&system);config(unavailable);unavailable.setup();unavailable.loop();assert(!unavailable._enabled&&unavailable._error);
 }
}
