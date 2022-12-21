/*

@type NetworkInterface
@description Base class for network interfaces

*/
#ifndef NETWORK_INTERFACE_MODULE_H
#define NETWORK_INTERFACE_MODULE_H

#include "../DroneModule.h"
#include "../DroneMeshMsg.h"
#include <ESPAsyncWebServer.h>



// class
class NetworkInterfaceModule:  public DroneModule {
protected:

public:

  NetworkInterfaceModule(uint8_t id, DroneSystem* ds);

  virtual void loop();

  virtual uint8_t getInterfaceType();
  virtual boolean getInterfaceState();

  // inherited by network interface implementations
  virtual boolean sendPacket(uint8_t *buffer);
  virtual void receivePacket(uint8_t *buffer, uint8_t metric);

};

#endif
