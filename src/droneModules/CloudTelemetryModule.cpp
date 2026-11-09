#include "CloudTelemetryModule.h"
#include "../CloudTelemetryHttp.h"
#include "../DroneSystem.h"
#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <atomic>
#include <new>
#include <esp_heap_caps.h>

CloudTelemetryModule* CloudTelemetryModule::instance=nullptr;
// Worker owns its copied settings/snapshot/client. It never dereferences a DroneModule.
struct CloudTelemetryWorker {
  telemetry::HttpSettings settings;
  telemetry::Cycle cycle;
  std::atomic<bool> active{false},done{false},terminate{false};
  std::atomic<uint32_t> generation{0};
  TaskHandle_t task=nullptr;
  uint32_t jobGeneration=0,duration=0,stack=0,requests=0,admitted=0,failed=0;
  uint8_t uploader=0;size_t attempted=0;int status=0;
  bool busy=false; // main-thread only
};
namespace {
class EspHttp:public telemetry::HttpIo {
  CloudTelemetryWorker &state;
  WiFiClientSecure client;
public:
  explicit EspHttp(CloudTelemetryWorker &worker):state(worker) {
    client.setInsecure();client.setHandshakeTimeout(5);client.setTimeout(5);
  }
  uint32_t now() override {return millis();}
  bool running() override {return !state.terminate.load()&&state.active.load()&&state.generation.load()==state.jobGeneration;}
  bool connect(const char* host,uint16_t port) override {
    if(!running()) return false;
    if(client.connected()) return true;
    return client.connect(host,port,5000) && running();
  }
  size_t write(const uint8_t* bytes,size_t size) override {return running()?client.write(bytes,size):0;}
  int read() override {return running()&&client.available()?client.read():-1;}
  bool connected() override {return client.connected();}
  void wait() override {vTaskDelay(1);}
  void close() override {client.stop();}
};
void runWorker(void* pointer) {
  auto &state=*static_cast<CloudTelemetryWorker*>(pointer);
  {
    EspHttp io(state);
    for(;;) {
      ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
      if(state.terminate.load()) break;
      const uint32_t start=millis();state.attempted=0;state.requests=state.admitted=state.failed=0;state.status=0;
      for(size_t i=0;i<state.cycle.count && io.running();) {
        const size_t count=std::min(telemetry::BatchLimit,state.cycle.count-i);
        telemetry::PackedHttp http(io);
        auto outcome=http.sendBatch(state.settings,state.cycle,i,count,state.uploader);
        if(outcome.attempted) {state.attempted+=count;++state.requests;}
        state.status=outcome.status;
        if(outcome.admitted>=0) state.admitted+=outcome.admitted;
        if(!outcome.complete) {++state.failed;break;}
        i+=count;
      }
      io.close(); // No TLS connection remains idle between report cycles.
      state.duration=millis()-start;
      state.stack=uxTaskGetStackHighWaterMark(nullptr); // ESP-IDF reports bytes.
      state.done.store(true,std::memory_order_release);
    }
    io.close();
  }
  delete &state;
  vTaskDelete(nullptr);
}
void quoted(File* file,const char* value) {
  file->print('"');
  for(;*value;++value) {if(*value=='"'||*value=='\\') file->print('\\');file->print(*value);}
  file->println('"');
}
}
CloudTelemetryModule::CloudTelemetryModule(uint8_t id,DroneSystem* system):DroneModule(id,system) {
  setTypeName(F("CloudTelemetry"));initParams(6);
  const char* names[]={"reportInterval","requests","selection","memory","timing","flow"};
  for(size_t i=0;i<6;++i) {
    auto &p=_params[i];p.paramPriority=setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW,8+i);
    // Names are static flash literals on ESP32, whose PROGMEM is memory mapped.
    setParamName(reinterpret_cast<const __FlashStringHelper*>(names[i]),&p);
    p.paramTypeLength=_mgmtMsg.packParamLength(i==0,DRONE_LINK_MSG_TYPE_UINT32_T,i==0?4:(i==3?16:12));
    memset(p.data.uint8,0,16);
  }
  _params[0].data.uint32[0]=rate;
  if(instance) {config.invalidate("duplicate CloudTelemetry");instance->config.invalidate("duplicate CloudTelemetry");}
  else instance=this;
}
CloudTelemetryModule::~CloudTelemetryModule() {
  if(worker) {worker->active=false;++worker->generation;worker->terminate=true;xTaskNotifyGive(worker->task);worker=nullptr;}
  if(instance==this) instance=nullptr;
}
bool CloudTelemetryModule::isConfigurationString(const char* name) const {
  return !strcmp(name,"endpoint")||!strcmp(name,"domain")||!strcmp(name,"apiKey")||!strcmp(name,"subscriptions");
}
telemetry::Setting CloudTelemetryModule::configure(const char* name,const char* value) {return config.apply(name,value);}
void CloudTelemetryModule::configurationError() {config.invalidate("invalid config.ini setting");}
void CloudTelemetryModule::saveExtraConfiguration(File* file) {
  const char* names[]={"endpoint","domain","apiKey"};const char* values[]={config.endpoint,config.domain,config.apiKey};
  for(size_t i=0;i<3;++i) {file->print("  ");file->print(names[i]);file->print(" = ");quoted(file,values[i]);}
  for(size_t i=0;i<config.count;++i) {
    auto &s=config.selections[i];file->print("  subscriptions = ");file->print(s.node);file->print('>');file->print(s.channel);file->print('.');file->println(s.param);
  }
}
void CloudTelemetryModule::setup() {
  DroneModule::setup();
  rate=_params[0].data.uint32[0];
  if(!config.complete()||!telemetry::Report::validInterval(rate)||!_dlm->node()||_dlm->node()>=255) {
    Log.errorln("[CloudTelemetry %u] %s",_id,config.error?config.error:"missing settings, node address or invalid reportInterval");
    setError(1);disable();return;
  }
  if(_enabled) enable();
}
void CloudTelemetryModule::enable() {
  if(!_setupDone) {DroneModule::enable();return;}
  if(!config.complete()||_error||!_dlm->node()||_dlm->node()>=255) return;
  if(started) return;
  DroneModule::enable();started=true;lastService=maxGap=0;intervalChanged=false;report.start(millis(),rate);
  if(worker) worker->active=true;
  lastResolve=millis()-1000;
}
void CloudTelemetryModule::waiting() {enable();}
void CloudTelemetryModule::disable() {
  started=false;report.stop();
  if(worker) {worker->active=false;++worker->generation;}
  DroneModule::disable();
}
void CloudTelemetryModule::doShutdown() {disable();}
bool CloudTelemetryModule::ensureWorker() {
  if(worker) return true;
  worker=new(std::nothrow) CloudTelemetryWorker;
  if(!worker) return false;
  worker->settings=config;
  if(xTaskCreate(runWorker,"cloudTelemetry",16384,worker,1,&worker->task)!=pdPASS) {delete worker;worker=nullptr;return false;}
  worker->active=true;return true;
}
void CloudTelemetryModule::resolve() {
  for(size_t i=0;i<config.count;++i) {
    auto &s=config.selections[i];if(s.subscribed) continue;
    unsigned node=0,channel=0,param=0;
    if(!strcmp(s.node,"@")) node=_dlm->node();
    else if(!telemetry::decimal(s.node,254,node)) node=_dlm->getNodeByName(s.node);
    if(!node||node>=255) continue;
    if(!telemetry::decimal(s.channel,254,channel)) {
      auto module=_dmm->getModuleByName(s.channel);if(!module) continue;channel=module->id();
    }
    if(strcmp(s.param,"*") && !telemetry::decimal(s.param,63,param)) {
      auto module=_dmm->getModuleById(channel);if(!module) continue;param=module->getParamIdByName(s.param);if(!param) continue;
    }
    s.resolvedNode=node;s.resolvedChannel=channel;s.resolvedParam=param;s.resolved=true;
    _dlm->subscribe(node,channel,this,param);s.subscribed=true;
  }
}
void CloudTelemetryModule::handleLinkMessage(DroneLinkMsg* message) {
  // Validate before legacy parameter handling; firmware callbacks may carry commands too.
  if(!message) return;
  DroneLinkMsg checked;
  const size_t length=5+message->length();
  if(!checked.tryParse(reinterpret_cast<const uint8_t*>(&message->_msg),length)) return;
  if(message->node()==_dlm->node()&&message->channel()==_id&&message->param()==8&&!message->writable()
      &&(message->type()!=DRONE_LINK_MSG_TYPE_UINT32_T||message->length()!=4)) return;
  DroneModule::handleLinkMessage(message);
  if(!started||!_enabled||_error||(message->node()==_dlm->node()&&message->channel()==_id)) return;
  for(size_t i=0;i<config.count;++i) {
    const auto &s=config.selections[i];
    if(s.resolved&&s.resolvedNode==message->node()&&s.resolvedChannel==message->channel()&&(!s.resolvedParam||s.resolvedParam==message->param())) {
      report.capture(reinterpret_cast<const uint8_t*>(&checked._msg),length);break;
    }
  }
}
void CloudTelemetryModule::onParamWrite(DRONE_PARAM_ENTRY* param) {
  if(param==&_params[0]) {
    if(param->paramTypeLength!=_mgmtMsg.packParamLength(true,DRONE_LINK_MSG_TYPE_UINT32_T,4)||!telemetry::Report::validInterval(param->data.uint32[0])) {
      param->paramTypeLength=_mgmtMsg.packParamLength(true,DRONE_LINK_MSG_TYPE_UINT32_T,4);param->data.uint32[0]=rate;return;
    }
    rate=param->data.uint32[0];intervalChanged=true;
    if(!worker||!worker->busy) {report.setInterval(millis(),rate);intervalChanged=false;}
  } else DroneModule::onParamWrite(param);
}
void CloudTelemetryModule::loop() {
  DroneModule::loop();const uint32_t now=millis();
  if(lastService) maxGap=std::max(maxGap,now-lastService);lastService=now;
  if(worker&&worker->busy&&worker->done.load(std::memory_order_acquire)) {
    const bool current=report.finish(worker->cycle,worker->attempted);worker->busy=false;
    if(current) {
      _params[1].data.uint32[0]+=worker->requests;_params[1].data.uint32[1]+=worker->admitted;_params[1].data.uint32[2]+=worker->failed;
      _params[3].data.uint32[2]=worker->stack;
      _params[4].data.uint32[1]=worker->duration;_params[4].data.uint32[2]=worker->status;
    }
  }
  if(worker&&worker->busy) report.skipTicks(now);
  if(intervalChanged&&(!worker||!worker->busy)) {report.setInterval(now,rate);intervalChanged=false;}

  if(uint32_t(now-lastResolve)>=1000) {resolve();lastResolve=now;}
  if(!worker||!worker->busy) {
    if(!ensureWorker()) {Log.errorln("[CloudTelemetry] worker allocation failed");setError(1);disable();return;}
    if(report.begin(now,WiFi.status()==WL_CONNECTED,worker->cycle)) {
      if(!_dlm->node()||_dlm->node()>=255) {Log.errorln("[CloudTelemetry] invalid uploader address");setError(1);disable();return;}
      worker->uploader=_dlm->node();worker->jobGeneration=worker->generation.load();worker->done=false;worker->busy=true;
      xTaskNotifyGive(worker->task);
    }
  }
  if(uint32_t(now-lastDiagnostics)>=5000) {
    size_t resolved=0;for(size_t i=0;i<config.count;++i) if(config.selections[i].resolved) ++resolved;
    _params[2].data.uint32[0]=resolved;_params[2].data.uint32[1]=config.count-resolved;_params[2].data.uint32[2]=report.overflow;
    _params[3].data.uint32[0]=ESP.getFreeHeap();_params[3].data.uint32[1]=ESP.getMinFreeHeap();_params[3].data.uint32[3]=heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    _params[5].data.uint32[0]=report.captured;_params[5].data.uint32[1]=report.coalesced;_params[5].data.uint32[2]=report.tracked();
    _params[4].data.uint32[0]=maxGap;
    for(size_t i=1;i<6;++i) publishParamEntry(&_params[i]);
    lastDiagnostics=now;
  }
}
