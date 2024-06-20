/*

@type NetworkInterface
@description Base class for network interfaces

*/
#ifndef NETWORK_INTERFACE_MODULE_H
#define NETWORK_INTERFACE_MODULE_H

#include "../DroneModule.h"
#include "../DroneMeshMsg.h"
#include <ESPAsyncWebServer.h>
#include "../DroneLinkManagerStructs.h"


// class
class NetworkInterfaceModule:  public DroneModule {
protected:
  boolean _broadcastCapable;
  uint8_t _peerId;  // ID of peer node on the other end of a point to point link

public:

  NetworkInterfaceModule(uint8_t id, DroneSystem* ds);

  virtual void loop();

  virtual uint8_t getInterfaceType();
  virtual boolean getInterfaceState();

  boolean isBroadcastCapable();

  void setPeerId(uint8_t id);
  uint8_t getPeerId();

  // inherited by network interface implementations
  virtual boolean sendPacket(uint8_t *buffer, DRONE_LINK_TRANSPORT_ADDRESS transportAddress);
  virtual void receivePacket(uint8_t *buffer, uint8_t metric, DRONE_LINK_TRANSPORT_ADDRESS transportAddress);

};

#endif
