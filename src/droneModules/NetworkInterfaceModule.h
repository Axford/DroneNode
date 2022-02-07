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
  boolean _interfaceState;
public:

  NetworkInterfaceModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  virtual void loop();

  boolean getInterfaceState();
  virtual uint8_t getInterfaceType();

  // inherited by network interface implementations
  virtual boolean sendPacket(uint8_t *buffer);
  virtual void receivePacket(uint8_t *buffer, uint8_t metric);

};

#endif
