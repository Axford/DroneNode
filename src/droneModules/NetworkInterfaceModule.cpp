#include "NetworkInterfaceModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

NetworkInterfaceModule::NetworkInterfaceModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   _interfaceState = false;  // start inactive
   _broadcastCapable = false;  // over-ridden by impl classes
   _peerId = 0;
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


boolean NetworkInterfaceModule::isBroadcastCapable() {
  return _broadcastCapable;
}


void NetworkInterfaceModule::setPeerId(uint8_t id) {
  _peerId = id;
}


uint8_t NetworkInterfaceModule::getPeerId() {
  return _peerId;
}


boolean NetworkInterfaceModule::sendPacket(uint8_t *buffer, DRONE_LINK_TRANSPORT_ADDRESS transportAddress) {
  // to be inherited
  return false;
}


void NetworkInterfaceModule::receivePacket(uint8_t *buffer, uint8_t metric, DRONE_LINK_TRANSPORT_ADDRESS transportAddress) {
  // pass onto DLM for processing
  _dlm->receivePacket(this, buffer, metric, transportAddress);
}
