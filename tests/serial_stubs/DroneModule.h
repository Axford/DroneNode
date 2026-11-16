#pragma once
#include "Arduino.h"
#define DRONE_LINK_MSG_PRIORITY_LOW 0
#define DRONE_LINK_MSG_TYPE_FLOAT 2
#define DRONE_LINK_MSG_TYPE_UINT8_T 0
#define DRONE_LINK_MSG_TYPE_UINT32_T 1
#define DRONE_MESH_INTERFACE_TYPE_SERIAL 2
#define DRONE_MESH_MSG_MAX_PACKET_SIZE 56
struct DroneSystem;
struct DRONE_LINK_TRANSPORT_ADDRESS {};
struct DRONE_PARAM_ENTRY {int paramPriority=0,paramTypeLength=0;union {uint8_t uint8[16];uint32_t uint32[4];float f[4];} data={};};
struct Mgmt {int packParamLength(bool,int,int){return 0;}};
inline int setDroneLinkMsgPriorityParam(int,int){return 0;}
class DroneModule {
 public:
  DroneSystem* _ds; bool _enabled=true; DRONE_PARAM_ENTRY _params[4]; Mgmt _mgmtMsg;
  DroneModule(uint8_t,DroneSystem* ds):_ds(ds){}
  void setTypeName(const char*){} void initParams(int){} void setParamName(const char*,DRONE_PARAM_ENTRY*){}
  void setError(int){} void publishParamEntry(DRONE_PARAM_ENTRY*){} void setup(){} void loop(){}
};
