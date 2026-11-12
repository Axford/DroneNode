#include "DroneModule.h"
#include "droneModules/CloudTelemetryModule.h"
#include "ConfigurationLine.h"
#include <cassert>
#include "actual_save.inc"
uint32_t hostTime=0;Logger Log;ESPClass ESP;thread_local Task* currentTask=nullptr;bool taskAllocationFails=false;
std::string requestBytes,responseBytes,configText;std::function<void()> duringWrite;HostFS LITTLEFS;
#include "WiFi.h"
WiFiClass WiFi;
class DroneExecutionManager {
public:
 DroneModuleManager* _dmm;DroneLinkManager* _dlm;DroneSystem system;std::vector<DroneModule*> modules;
 DroneExecutionManager(DroneModuleManager* dmm,DroneLinkManager* dlm):_dmm(dmm),_dlm(dlm),system{dlm,dmm}{}
 ~DroneExecutionManager(){for(auto module:modules)delete module;}
 DroneModule* instanceModule(char* type,unsigned id){
  if(strcmp(type,"CloudTelemetry")&&strcmp(type,"Other"))return nullptr;
  DroneModule* m=!strcmp(type,"CloudTelemetry")?static_cast<DroneModule*>(new CloudTelemetryModule(id,&system)):new DroneModule(id,&system);
  modules.push_back(m);return m;
 }
 void addToAddressQueue(DroneModule* module,char*,char* value){module->legacyLength=strlen(value);}
 void loadConfiguration(const char* filename);
};
#define DEM_PARSER_VALUE_BUFFER_SIZE 200
#include "actual_loader.inc"
#define DRONE_LINK_PARAM_ALL 0
struct DroneLinkChannelSubscription {DroneModule* module;unsigned param,extNode;};
template<class T>struct Queue {std::vector<T> values;int size(){return values.size();}T shift(){auto v=values.front();values.erase(values.begin());return v;}T get(unsigned i){return values[i];}void remove(unsigned i){values.erase(values.begin()+i);}};
struct DroneLinkQueueEntry {DroneLinkMsg message;bool cloudRequest;};
class DroneLinkChannel {public:unsigned _cloudRequests=0,_cloudWaitTurns=0;Queue<DroneLinkQueueEntry*> _queue;Queue<DroneLinkChannelSubscription*> _subs;DroneLinkManager* _dlm;void processQueue();};
#include "actual_channel.inc"
struct Counter:DroneModule{unsigned count=0;Counter(DroneSystem* ds):DroneModule(1,ds){}void handleLinkMessage(DroneLinkMsg*)override{++count;}};
int main(){
 std::string savedCloud;
 DroneModuleManager dmm;DroneLinkManager dlm;
 {
  DroneSystem system{&dlm,&dmm};Counter counter(&system);DroneLinkChannel channel;channel._dlm=&dlm;
  DroneLinkChannelSubscription all{&counter,0,11},specific{&counter,8,11},external{nullptr,8,254};
  channel._subs.values={&all,&specific,&external};
  for(unsigned i=0;i<2;++i){auto message=new DroneLinkQueueEntry{};message->message._msg.param=8;channel._queue.values.push_back(message);channel.processQueue();}
  assert(counter.count==2&&dlm.external.size()==2);
  Counter observer(&system);observer._id=2;
  DroneLinkChannelSubscription observed{&observer,0,11};
  channel._subs.values.insert(channel._subs.values.begin(),&observed);
  auto injected=new DroneLinkQueueEntry{};
  injected->message._msg.channel=1;injected->message._msg.param=8;injected->cloudRequest=true;
  channel._queue.values.push_back(injected);channel._cloudRequests=1;channel.processQueue();
  assert(counter.count==3&&observer.count==0&&dlm.external.size()==2&&channel._cloudRequests==0);
  for(unsigned i=0;i<5;++i) channel._queue.values.push_back(new DroneLinkQueueEntry{});
  auto query=new DroneLinkQueueEntry{};
  query->message._msg.channel=1;query->message._msg.param=8;query->cloudRequest=true;
  channel._queue.values.push_back(query);channel._cloudRequests=1;
  for(unsigned i=0;i<4;++i) channel.processQueue();
  assert(channel._cloudRequests==0&&channel._queue.size()==2);
  while(channel._queue.size())channel.processQueue();
 }

 {
  DroneExecutionManager loader(&dmm,&dlm);
  configText="node=11\n[Unknown=1]\nnode=12\n[Other=2]\nnode=13\n";
  loader.loadConfiguration("/config.ini");assert(dlm.node()==11&&dmm.address==11);
 }
 {
  DroneExecutionManager loader(&dmm,&dlm);
  const std::string key(512,'a');
  configText="[CloudTelemetry=20]\nendpoint=https://example.test\ndomain=fixture\napiKey="+key+"\nsubscriptions=@>1.*\nsubscriptions=Hull>4.8\npublish=reportInterval";
  loader.loadConfiguration("/config.ini");assert(loader.modules.size()==1);
  auto module=loader.modules[0];module->setup();assert(module->_enabled&&!module->_error);
  File saved;module->saveConfiguration(&saved);savedCloud=saved.text;assert(saved.text.find("publish = reportInterval\n  endpoint")!=std::string::npos);assert(saved.text.find(key)!=std::string::npos&&saved.text.find("Hull>4.8")!=std::string::npos);
 }
 {
  DroneExecutionManager loader(&dmm,&dlm);configText=savedCloud;loader.loadConfiguration("/config.ini");loader.modules[0]->setup();assert(loader.modules[0]->_enabled&&!loader.modules[0]->_error);
 }
 {
  DroneExecutionManager loader(&dmm,&dlm);
  configText="[Other=1]\npublish="+std::string(260,'x')+"\n";
  loader.loadConfiguration("/config.ini");assert(loader.modules[0]->legacyLength==0);
 }
 {
  DroneExecutionManager loader(&dmm,&dlm);
  configText="[Other=1]\nname=\\literal\npublish=status, name\n";
  loader.loadConfiguration("/config.ini");assert(loader.modules[0]->legacyLength==11);
 }
}
