/*

@type NetworkInterface
@description Base class for network interfaces

*/
#ifndef NETWORK_INTERFACE_MODULE_H
#define NETWORK_INTERFACE_MODULE_H

#include "../DroneModule.h"
#include "../DroneMeshMsg.h"

#define NETWORK_INTERFACE_MAX_TX_QUEUE    8


// class
class NetworkInterfaceModule:  public DroneModule {
protected:
  boolean _interfaceState;
  uint8_t _helloSeq;
  IvanLinkedList::LinkedList<DRONE_MESH_MSG_BUFFER*> _txQueue;
public:

  NetworkInterfaceModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  virtual void loop();

  boolean getInterfaceState();
  DRONE_MESH_MSG_BUFFER* getTransmitBuffer();
  void processTransmitQueue();

  void generateHello();

  // inherited by network interface implementations
  virtual boolean sendPacket(uint8_t *buffer);
};

#endif
