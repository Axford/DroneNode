/*

@type UDPTelemetry
@description Manages DroneLink telemetry using UDP broadcast over WiFi

@config >>>
UDPTelemetry.new 2
  name "UDPT"
  status 1 // enable
  port 8007
  broadcast 255 255 255 255
  .sub [@>0.0]
.done
<<<

Publishes received messages
Re-transmits messages it has been subscribed to
Handles packet framing, sync, etc

*/
#ifndef UDP_TELEMETRY_MODULE_H
#define UDP_TELEMETRY_MODULE_H

#include "Arduino.h"
#include "../DroneModule.h"
#include "./NetworkInterfaceModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneMeshMsg.h"
#include <WiFiUdp.h>
/*

Packet framing:

byte    = value
[0...n] = DroneLinkMsg raw data
*/


#define UDP_PARAM_PORT               8
#define UDP_PARAM_BROADCAST          9

#define UDP_PARAM_PORT_E             0
#define UDP_PARAM_BROADCAST_E        1

// @pub 10;u32;3;packets;Packet counters for sent, received and rejected
#define UDP_TELEMETRY_PARAM_PACKETS        10
#define UDP_TELEMETRY_PARAM_PACKETS_E      2

// @pub 11;f;3;speed;Packet rates per second for sent, received and rejected
#define UDP_TELEMETRY_PARAM_SPEED          11
#define UDP_TELEMETRY_PARAM_SPEED_E        3

#define UDP_PARAM_ENTRIES                  4


#define UDP_TELEMETRY_PORT   8007

static const char UDP_TELEMETRY_STR_UDP_TELEMETRY[] PROGMEM = "UDPTelemetry";


class UDPTelemetryModule:  public NetworkInterfaceModule {
protected:
  WiFiUDP _udp;
  uint8_t _rBuffer[DRONE_MESH_MSG_MAX_PACKET_SIZE];

  uint8_t _receivedSize;
  boolean _started;

  uint32_t _packetsReceived;
  uint32_t _packetsRejected;
  uint32_t _packetsSent;
  unsigned long _packetsTimer;
public:

  UDPTelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);


  uint8_t getInterfaceType();

  virtual void setup();
  virtual void loop();

  // network interface methods
  boolean sendPacket(uint8_t *buffer);
};

#endif
