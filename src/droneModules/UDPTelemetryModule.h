/*

@type          UDPTelemetry
@category      Networking
@description   Manages DroneLink telemetry using UDP broadcast over WiFi
@inherits      Drone

@config >>>
[UDPTelemetry=2]
  name=UDPT
  port =8007
  broadcast =255, 255, 255, 255
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

// @pub 8;u32;1;w;port;UDP port to broadcast to, default 8007
#define UDP_PARAM_PORT               8

// @pub 9;u8;4;w;broadcast;IP address to broadcast to, default: 255,255,255,255
#define UDP_PARAM_BROADCAST          9

#define UDP_PARAM_PORT_E             0
#define UDP_PARAM_BROADCAST_E        1

// @pub 10;u32;3;r;packets;Packet counters for sent, received and rejected
#define UDP_TELEMETRY_PARAM_PACKETS        10
#define UDP_TELEMETRY_PARAM_PACKETS_E      2

// @pub 11;f;3;r;speed;Packet rates per second for sent, received and rejected
#define UDP_TELEMETRY_PARAM_SPEED          11
#define UDP_TELEMETRY_PARAM_SPEED_E        3

// @pub 12;c;15;w;URL;URL for a remote server
#define UDP_TELEMETRY_PARAM_URL            12
#define UDP_TELEMETRY_PARAM_URL_E          4

#define UDP_PARAM_ENTRIES                  5


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

  IPAddress _serverAddress;  // resolved from URL
public:

  UDPTelemetryModule(uint8_t id, DroneSystem* ds);

  uint8_t getInterfaceType();

  virtual void setup();
  virtual void loop();

  // network interface methods
  boolean sendPacket(uint8_t *buffer);
};

#endif
