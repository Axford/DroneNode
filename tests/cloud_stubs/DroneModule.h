#pragma once
#include "CloudTelemetryConfig.h"
#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <stdint.h>
#include <string>
#include <vector>
#include <functional>
#include <stdexcept>
using boolean=bool;
using __FlashStringHelper=char;using PGM_P=const char*;
#define DRONE_MODULE_PARAM_TYPE_E 4
#define DRONE_MGMT_PARAM_ENTRIES 6
#define F(x) x
#define DRONE_LINK_MSG_PRIORITY_LOW 0
#define DRONE_LINK_MSG_TYPE_UINT32_T 2
#define DRONE_LINK_MSG_TYPE_UINT8_T 0
extern uint32_t hostTime;
inline uint32_t millis(){return hostTime;}
inline uint32_t micros(){return hostTime*1000;}
struct File {
 std::string text;size_t position=0;bool valid=true;
 explicit operator bool()const{return valid;}int available(){return text.size()-position;}int read(){return available()?text[position++]:-1;}void close(){}
 void print(const char* s){text+=s;}void print(char c){text+=c;}void print(unsigned v){text+=std::to_string(v);}void print(int v){text+=std::to_string(v);}
 void println(char c){print(c);text+='\n';}void println(const char* s){print(s);text+='\n';}
};
struct Logger {template<class...Args>void errorln(Args...){} };extern Logger Log;
union Payload {uint8_t uint8[16];uint32_t uint32[4];float f[4];};
struct DRONE_PARAM_ENTRY {uint8_t paramPriority=0,paramTypeLength=0;Payload data={};const char* name="";bool publish=false;};
struct DRONE_PARAM_SUB {struct {unsigned node=0,channel=0,paramPriority=0;}addr;DRONE_PARAM_ENTRY param;};
struct Raw {uint8_t source=0,node=0,channel=0,param=0,typed=0;Payload payload={};} __attribute__((packed));
struct DroneLinkMsg {
 Raw _msg;
 uint8_t packParamLength(bool writable,unsigned type,unsigned length){return (writable?128:0)|(type<<4)|(length-1);}
 unsigned length()const{return (_msg.typed&15)+1;}unsigned type()const{return (_msg.typed>>4)&7;}
 unsigned node()const{return _msg.node;}unsigned channel()const{return _msg.channel;}unsigned param()const{return _msg.param&63;}
 bool writable()const{return _msg.typed&128;}
 bool tryParse(const uint8_t* p,size_t n){if(!p||n<6||n>21||n!=size_t(5+(p[4]&15)+1))return false;unsigned type=(p[4]>>4)&7;if(type>=1&&type<=3&&((p[4]&15)+1)%4)return false;memset(&_msg,0,sizeof _msg);memcpy(&_msg,p,n);return true;}
};
inline unsigned setDroneLinkMsgPriorityParam(unsigned,unsigned p){return p;}
struct DroneSystem;
struct DroneLinkManager;
struct DroneModuleManager;
class DroneModule {
public:
 DroneSystem* _ds;DroneLinkManager* _dlm;DroneModuleManager* _dmm;
 uint8_t _id;bool _enabled=true,_setupDone=false;uint8_t _error=0;
 DRONE_PARAM_ENTRY _params[6],_mgmtParams[6];unsigned _numParamEntries=0,_numSubs=0;DRONE_PARAM_SUB _subs[1];char typeName[17]={};DroneLinkMsg _mgmtMsg;
 DroneModule(uint8_t id,DroneSystem* ds);
 virtual ~DroneModule(){}
 void setTypeName(const char* name){strncpy(typeName,name,16);}void initParams(unsigned n){_numParamEntries=n;}void setParamName(const char* name,DRONE_PARAM_ENTRY* p){p->name=name;}
 char* getName(){return typeName;}DRONE_PARAM_ENTRY* getParamEntryById(unsigned id){return id>=8&&id<14?&_params[id-8]:nullptr;}
 void saveParamValue(DRONE_PARAM_ENTRY*,File* f){f->print(typeName);}void saveParam(DRONE_PARAM_ENTRY*,File*){}
 virtual bool isConfigurationString(const char*)const{return false;}
 virtual telemetry::Setting configure(const char*,const char*){return telemetry::Setting::Unhandled;}
 virtual void configurationError(){}virtual void saveExtraConfiguration(File*){}
 virtual void setup(){_setupDone=true;}virtual void loop(){}virtual void onParamWrite(DRONE_PARAM_ENTRY*){}
 virtual void enable(){_enabled=true;}virtual void disable(){_enabled=false;}virtual void waiting(){}virtual void doShutdown(){}
 virtual void handleLinkMessage(DroneLinkMsg* m){if(m->channel()==_id&&m->param()==8&&!m->writable()&&m->type()==2&&m->length()==4){_params[0].data=m->_msg.payload;_params[0].paramTypeLength=m->_msg.typed|128;onParamWrite(&_params[0]);}}
 size_t legacyLength=0;
 void publishParamsFromList(char* p){legacyLength=strlen(p);if(strstr(p,"reportInterval"))_params[0].publish=true;}
 DRONE_PARAM_ENTRY* getParamEntryByName(const char*){return &_params[0];}
 void setParamFromList(DRONE_PARAM_ENTRY*,char* p){legacyLength=strlen(p);}
 void saveConfiguration(File* f);
 void setError(unsigned value){_error=value;}void publishParamEntry(DRONE_PARAM_ENTRY*){}
 uint8_t id(){return _id;}uint8_t getParamIdByName(const char* name){return !strcmp(name,"location")?8:0;}
};
struct DroneLinkManager {
 struct Sub{unsigned node,channel,param;};std::vector<Sub> subs;bool hull=false;
 std::vector<unsigned> external;void sendDroneLinkMessage(unsigned n,DroneLinkMsg*){external.push_back(n);}
 unsigned address=11;unsigned node(){return address;}void node(unsigned n){address=n;}unsigned getNodeByName(const char* name){return hull&&!strcmp(name,"Hull")?12:0;}
 void subscribe(unsigned n,unsigned c,DroneModule*,unsigned p){subs.push_back({n,c,p});}
};
struct DroneModuleManager {
 unsigned address=11;void node(unsigned n){address=n;}unsigned node(){return address;}
 DroneModule* gps=nullptr;
 DroneModule* getModuleByName(const char* name){return !strcmp(name,"GPS")?gps:nullptr;}
 DroneModule* getModuleById(unsigned id){return id==4?gps:nullptr;}
};
struct DroneSystem {DroneLinkManager* dlm;DroneModuleManager* dmm;};
inline DroneModule::DroneModule(uint8_t id,DroneSystem* ds):_ds(ds),_dlm(ds->dlm),_dmm(ds->dmm),_id(id){}
struct Task;
using TaskHandle_t=Task*;
struct Task {void(*run)(void*);void* state;unsigned waits;Task(void(*fn)(void*),void* p):run(fn),state(p),waits(0){}};
extern thread_local Task* currentTask;
struct Pause{};struct Finished{};
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY 0xffffffff
extern bool taskAllocationFails;
inline int xTaskCreate(void(*run)(void*),const char*,unsigned,void* state,unsigned,TaskHandle_t* task){if(taskAllocationFails)return 0;*task=new Task(run,state);return 1;}
inline void xTaskNotifyGive(Task* task){task->waits=0;currentTask=task;try{task->run(task->state);}catch(Pause&){}catch(Finished&){delete task;}currentTask=nullptr;}
inline unsigned ulTaskNotifyTake(int,unsigned){if(currentTask->waits++)throw Pause();return 1;}
inline void vTaskDelete(void*){throw Finished();}
inline void vTaskDelay(unsigned n){hostTime+=n;}
inline unsigned uxTaskGetStackHighWaterMark(void*){return 8192;}
struct ESPClass{unsigned getFreeHeap(){return 100000;}unsigned getMinFreeHeap(){return 90000;}};extern ESPClass ESP;

#define FILE_READ 0
extern std::string configText;
struct HostFS{File open(const char*,int){File f;f.text=configText;return f;}};
extern HostFS LITTLEFS;
inline void yield(){}
