#pragma once
#include "DroneModule.h"
#include "DroneLinkManager.h"
extern std::vector<std::vector<uint8_t>> handoffs;
class NetworkInterfaceModule: public DroneModule {
 public:
  DroneLinkManager manager; DroneLinkManager* _dlm=&manager; bool _broadcastCapable=false,_interfaceState=false;
  NetworkInterfaceModule(uint8_t id,DroneSystem* ds):DroneModule(id,ds){}
  void receivePacket(uint8_t* b,int,DRONE_LINK_TRANSPORT_ADDRESS){handoffs.emplace_back(b,b+((b[0]&63)+9));}
};
