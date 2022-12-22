#include "NetworkInterfaceModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

NetworkInterfaceModule::NetworkInterfaceModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   _interfaceState = false;  // start inactive
   Log.noticeln("[NIM]");
}


void NetworkInterfaceModule::loop() {
  DroneModule::loop();

}


boolean NetworkInterfaceModule::getInterfaceState() {
  return _interfaceState && _enabled;
}


uint8_t NetworkInterfaceModule::getInterfaceType() {
  // to be overridden
  return DRONE_MESH_INTERFACE_TYPE_UDP;
}


boolean NetworkInterfaceModule::sendPacket(uint8_t *buffer) {
  // to be inherited
  return false;
}


void NetworkInterfaceModule::receivePacket(uint8_t *buffer, uint8_t metric) {
  // pass onto DLM for processing
  _dlm->receivePacket(this, buffer, metric);
}
