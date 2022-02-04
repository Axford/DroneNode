/*

@type NetworkInterface
@description Base class for network interfaces

*/
#ifndef NETWORK_INTERFACE_MODULE_H
#define NETWORK_INTERFACE_MODULE_H

#include "../DroneModule.h"
#include "../DroneMeshMsg.h"
#include <FastCRC.h>
#include <ESPAsyncWebServer.h>

#define NETWORK_INTERFACE_MAX_TX_QUEUE    8

#define NETWORK_INTERFACE_HELLO_INTERVAL  5000
#define NETWORK_INTERFACE_SEQ_INTERVAL    30000

// class
class NetworkInterfaceModule:  public DroneModule {
protected:
  boolean _interfaceState;
  uint8_t _helloSeq;
  IvanLinkedList::LinkedList<DRONE_MESH_MSG_BUFFER*> _txQueue;
  FastCRC8 _CRC8;
  uint32_t _helloTimer;
  uint32_t _seqTimer;
public:

  NetworkInterfaceModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  virtual void loop();

  boolean getInterfaceState();
  uint8_t getTxQueueSize();
  DRONE_MESH_MSG_BUFFER* getTransmitBuffer();
  void processTransmitQueue();

  boolean generateNextHop(uint8_t *buffer, uint8_t nextHop);

  void generateHello();
  boolean generateHello(uint8_t src, uint8_t seq, uint8_t metric, uint32_t uptime);

  boolean generateSubscriptionRequest(uint8_t src, uint8_t next, uint8_t dest, uint8_t channel, uint8_t param);

  boolean generateTraceroute(uint8_t destNode, uint8_t nextNode);

  boolean generateRouteEntryResponse(void * nodeInfo, uint8_t target, uint8_t dest, uint8_t nextHop);

  boolean sendDroneLinkMessage(uint8_t destNode, uint8_t nextNode, DroneLinkMsg *msg);

  // inherited by network interface implementations
  virtual boolean sendPacket(uint8_t *buffer);
  virtual void receivePacket(uint8_t *buffer, uint8_t metric);

  void serveTxQueueInfo(AsyncResponseStream* response);
};

#endif
