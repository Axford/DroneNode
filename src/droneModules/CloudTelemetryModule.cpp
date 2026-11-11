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
enum class CloudJob : uint8_t { Report, Prompt, Poll };
constexpr uint32_t JobSignal = 1, StopSignal = 2;

struct CloudTelemetryWorker {
  telemetry::HttpSettings settings;
  telemetry::Cycle cycle;
  telemetry::CommandBatch batch;
  telemetry::PollPeers peers;
  CloudJob job = CloudJob::Report;
  size_t pollLimit = 0;
  std::atomic<bool> active{false},done{false};
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
  uint32_t nowMicros() override {return micros();}
  void observe(telemetry::Phase phase,bool end,uint32_t elapsed) override {telemetry::measurePhase(phase,end,elapsed);}
  bool running() override {return state.active.load()&&state.generation.load()==state.jobGeneration;}
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
      uint32_t signals=0;
      xTaskNotifyWait(0,UINT32_MAX,&signals,portMAX_DELAY);
      if(signals&StopSignal) break;
      telemetry::measurePhase(telemetry::Phase::Report,false,0);
      const uint32_t reportStart=micros();
      const uint32_t start=millis();state.attempted=0;state.requests=state.admitted=state.failed=0;state.status=0;
      if (state.job == CloudJob::Poll) {
        telemetry::PackedHttp http(io);
        auto outcome = http.poll(state.settings, state.uploader, state.pollLimit, state.batch, state.peers);
        state.requests = outcome.attempted;
        state.failed = !outcome.complete;
        state.status = outcome.status;
      } else for(size_t i=0;i<state.cycle.count && io.running();) {
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
      telemetry::measurePhase(telemetry::Phase::Report,true,micros()-reportStart);
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
  setTypeName(F("CloudTelemetry"));initParams(7);
  const char* names[]={"reportInterval","requests","selection","memory","timing","flow","pollInterval"};
  for(size_t i=0;i<7;++i) {
    auto &p=_params[i];p.paramPriority=setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW,8+i);
    // Names are static flash literals on ESP32, whose PROGMEM is memory mapped.
    setParamName(reinterpret_cast<const __FlashStringHelper*>(names[i]),&p);
    const bool writable=i==0||i==6;
    p.paramTypeLength=_mgmtMsg.packParamLength(writable,DRONE_LINK_MSG_TYPE_UINT32_T,writable?4:(i==3?16:12));
    memset(p.data.uint8,0,16);
  }
  _params[0].data.uint32[0]=rate;
  _params[6].data.uint32[0]=pollRate;
  if(instance) {config.invalidate("duplicate CloudTelemetry");instance->config.invalidate("duplicate CloudTelemetry");}
  else instance=this;
}
CloudTelemetryModule::~CloudTelemetryModule() {
  if(worker) {
    worker->active=false;
    ++worker->generation;
    auto task=worker->task;
    worker=nullptr;
    xTaskNotify(task,StopSignal,eSetBits);
  }
  if(instance==this) {
    _dlm->observeLocalPublications(nullptr,nullptr);
    _dlm->discardCloudRequests();
    instance=nullptr;
  }
}
bool CloudTelemetryModule::isConfigurationString(const char* name) const {
  return !strcmp(name,"endpoint")||!strcmp(name,"domain")||!strcmp(name,"apiKey")||!strcmp(name,"subscriptions")||!strcmp(name,"relayPeers");
}
telemetry::Setting CloudTelemetryModule::configure(const char* name,const char* value) {
  if(strcmp(name,"relayPeers")) return config.apply(name,value);
  uint8_t selected[sizeof relayPeers] = {};
  unsigned count=0;
  while(value && *value) {
    unsigned peer=0;
    const char* start=value;
    while(*value>='0' && *value<='9' && peer<=254) peer=peer*10+(*value++-'0');
    if(value==start || !peer || peer>=255 || (*value && *value!=',') ||
       (selected[peer/8] & (1u<<(peer%8))) || ++count>64) break;
    selected[peer/8]|=1u<<(peer%8);
    if(!*value) {
      memcpy(relayPeers,selected,sizeof relayPeers);
      return telemetry::Setting::Handled;
    }
    ++value;
  }
  config.invalidate("invalid relayPeers");
  return telemetry::Setting::Invalid;
}
void CloudTelemetryModule::configurationError() {config.invalidate("invalid config.ini setting");}
void CloudTelemetryModule::saveExtraConfiguration(File* file) {
  bool first=true;
  for(unsigned peer=1;peer<255;++peer) if(relayPeers[peer/8] & (1u<<(peer%8))) {
    file->print(first?"  relayPeers = ":",");
    file->print(peer);
    first=false;
  }
  if(!first) file->println("");
  const char* names[]={"endpoint","domain","apiKey"};const char* values[]={config.endpoint,config.domain,config.apiKey};
  for(size_t i=0;i<3;++i) {file->print("  ");file->print(names[i]);file->print(" = ");quoted(file,values[i]);}
  for(size_t i=0;i<config.count;++i) {
    auto &s=config.selections[i];file->print("  subscriptions = ");file->print(s.node);file->print('>');file->print(s.channel);file->print('.');file->println(s.param);
  }
}
void CloudTelemetryModule::setup() {
  DroneModule::setup();
  rate=_params[0].data.uint32[0];
  pollRate=_params[6].data.uint32[0];
  if(pollRate && !telemetry::Report::validInterval(pollRate)) {
    Log.errorln("[CloudTelemetry %u] invalid pollInterval",_id);
    setError(1);disable();return;
  }
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
  nextPoll=millis()+pollRate;pollChanged=false;nextJob=0;
  _dlm->observeLocalPublications(this,observePublication);
  if(worker) worker->active=true;
  lastResolve=millis()-1000;
}
void CloudTelemetryModule::waiting() {enable();}
void CloudTelemetryModule::disable() {
  started=false;report.stop();
  batchReady=false;readback=false;peerWaiting=false;responses.clear();
  if(instance==this) {
    _dlm->observeLocalPublications(nullptr,nullptr);
    _dlm->discardCloudRequests();
  }
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
  if(message->node()==_dlm->node()&&message->channel()==_id&&(message->param()==8||message->param()==14)&&!message->writable()&&message->type()<5
      &&(message->type()!=DRONE_LINK_MSG_TYPE_UINT32_T||message->length()!=4)) return;
  DroneModule::handleLinkMessage(message);

}
void CloudTelemetryModule::captureConfigured(const uint8_t* bytes,size_t length) {
  if(!started||!_enabled||_error||(bytes[1]==_dlm->node()&&bytes[2]==_id)) return;
  for(size_t i=0;i<config.count;++i) {
    const auto &s=config.selections[i];
    if(s.resolved&&s.resolvedNode==bytes[1]&&s.resolvedChannel==bytes[2]&&(!s.resolvedParam||s.resolvedParam==(bytes[3]&63))) {
      report.capture(bytes,length);
      break;
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
  } else if (param==&_params[6]) {
    const auto shape=_mgmtMsg.packParamLength(true,DRONE_LINK_MSG_TYPE_UINT32_T,4);
    const uint32_t value=param->data.uint32[0];
    if(param->paramTypeLength!=shape||(value&&!telemetry::Report::validInterval(value))) {
      param->paramTypeLength=shape;param->data.uint32[0]=pollRate;return;
    }
    pollRate=value;pollChanged=true;
  } else DroneModule::onParamWrite(param);
}
void CloudTelemetryModule::observePublication(void* context,const DroneLinkMsg& message,bool directed) {
  auto& module=*static_cast<CloudTelemetryModule*>(context);
  if (!module.started || module._error) return;
  auto& raw=message._msg;
  const auto* bytes=reinterpret_cast<const uint8_t*>(&raw);
  const size_t length=6+(bytes[4]&15);
  if (module.responses.capture(bytes,length,millis())) module.report.discardOlder(bytes);
  else if (!directed) module.captureConfigured(bytes,length);
}
bool CloudTelemetryModule::waitForAdmission(CloudRequestResult result,bool peer,uint32_t now) {
  if (result!=CloudRequestResult::Busy) {peerWaiting=false;return false;}
  if (!peer) return true;
  if (!peerWaiting) {peerWaiting=true;peerWaitStart=now;}
  if (uint32_t(now-peerWaitStart)<5000) return true;
  peerWaiting=false;
  return false;
}
void CloudTelemetryModule::serviceCommands(uint32_t now) {
  if (!batchReady || _dlm->cloudRequestPending()) return;
  if (commandIndex==worker->batch.count && !readback) {
    batchReady=false;
    nextPoll=now+pollRate;
    return;
  }

  auto packet=worker->batch.packets[readback ? commandIndex-1 : commandIndex];
  if (readback) {
    packet.bytes[3]&=63;
    packet.bytes[4]=7<<4;
    packet.bytes[5]=0;
    packet.size=6;
  }
  DroneLinkMsg message;
  if (!message.tryParse(packet.bytes,packet.size)) {
    if (readback) readback=false;
    else ++commandIndex;
    return;
  }
  if (message.type()<5) {
    const auto result=_dlm->publishCloudRequest(message);
    if (waitForAdmission(result,message.node()!=_dlm->node(),now)) return;
    ++commandIndex;
    readback=result==CloudRequestResult::Queued;
    return;
  }
  auto* window=responses.select(packet,now);
  if (!window) return;
  const auto previous=*window;
  window->node=packet.bytes[1];window->channel=packet.bytes[2];
  window->param=packet.bytes[3]&63;window->facet=message.type()==6;
  window->deadline=now+30000;
  const auto result=_dlm->publishCloudRequest(message);
  if (result!=CloudRequestResult::Queued) *window=previous;
  if (waitForAdmission(result,message.node()!=_dlm->node(),now)) return;
  if (readback) readback=false;
  else ++commandIndex;
}
void CloudTelemetryModule::scheduleJob(uint32_t now) {
  if (!started || (worker && worker->busy)) return;
  if (!ensureWorker()) {Log.errorln("[CloudTelemetry] worker allocation failed");setError(1);disable();return;}
  const bool connected=WiFi.status()==WL_CONNECTED;
  if (pollChanged && !batchReady && !_dlm->cloudRequestPending()) {
    nextPoll=now+pollRate;
    pollChanged=false;
  }
  for (uint8_t i=0;i<3;++i) {
    const uint8_t index=(nextJob+i)%3;
    const auto job=static_cast<CloudJob>(index);
    bool work=false;
    switch(job) {
      case CloudJob::Report:
        work=report.begin(now,connected,worker->cycle,[this](const telemetry::Packet& p) { return !responses.pendingFacet(p); });
        break;
      case CloudJob::Prompt: work=connected && responses.take(worker->cycle);break;
      case CloudJob::Poll:
        if(pollRate && !pollChanged && !batchReady && int32_t(now-nextPoll)>=0 && !_dlm->cloudRequestPending()) {
          nextPoll=now+pollRate;
          worker->pollLimit=std::min(telemetry::BatchLimit,responses.capacity(now));
          work=connected && worker->pollLimit>0;
        }
        break;
    }
    if(!work) continue;
    if(!_dlm->node()||_dlm->node()>=255) {Log.errorln("[CloudTelemetry] invalid uploader address");setError(1);disable();return;}
    if(job==CloudJob::Poll) {
      worker->peers.count=0;
      {
        for(unsigned n=0;n<254 && worker->peers.count<telemetry::PeerPollLimit;++n) {
          const auto peer=peerCursor;
          peerCursor=peerCursor==254?1:peerCursor+1;
          if((relayPeers[peer/8] & (1u<<(peer%8))) && _dlm->hasCommandRoute(peer)) worker->peers.nodes[worker->peers.count++]=peer;
        }
      }
    }
    worker->job=job;
    worker->uploader=_dlm->node();worker->jobGeneration=worker->generation.load();worker->done=false;worker->busy=true;
    nextJob=(index+1)%3;
    xTaskNotify(worker->task,JobSignal,eSetBits);
    return;
  }
}
void CloudTelemetryModule::loop() {
  DroneModule::loop();const uint32_t now=millis();
  if(lastService) maxGap=std::max(maxGap,now-lastService);
  lastService=now;
  if(worker&&worker->busy&&worker->done.load(std::memory_order_acquire)) {
    const bool current=started && worker->jobGeneration==worker->generation.load();
    worker->busy=false;
    if(current) {
      if(worker->job==CloudJob::Report) report.finish(worker->cycle,worker->attempted);
      if(worker->job==CloudJob::Poll) {
        commandIndex=0;readback=false;
        batchReady=worker->batch.count>0 && worker->uploader==_dlm->node();
        nextPoll=now+pollRate;
      }
      _params[1].data.uint32[0]+=worker->requests;_params[1].data.uint32[1]+=worker->admitted;_params[1].data.uint32[2]+=worker->failed;
      _params[3].data.uint32[2]=worker->stack;
      _params[4].data.uint32[1]=worker->duration;_params[4].data.uint32[2]=worker->status;
    }
  }
  if(worker&&worker->busy&&worker->job==CloudJob::Report) report.skipTicks(now);
  if(intervalChanged&&(!worker||!worker->busy)) {report.setInterval(now,rate);intervalChanged=false;}
  if(!started) return;
  if(uint32_t(now-lastResolve)>=1000) {resolve();lastResolve=now;}
  serviceCommands(now);
  scheduleJob(now);
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
