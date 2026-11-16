/*

@type SerialTelemetry
@inherit       NetworkInterface
@category      Networking
@description Manages DroneLink telemetry over a serial port, optionally via a transparent telemetry module

@config >>>
SerialTelemetry.new 3
  name "serial"
  status 1  // enable
.done
<<<

*/

#ifndef SERIAL_TELEMETRY_MODULE_H
#define SERIAL_TELEMETRY_MODULE_H

#include "Arduino.h"
#include "../DroneModule.h"
#include "../DroneLinkMsg.h"
#include "./NetworkInterfaceModule.h"
#include "../SerialTransport.h"

#include "../pinConfig.h"

// @pub 8;u8;1;w;port;Serial port (0-2, default: 1)
#define SERIAL_TELEMETRY_PARAM_PORT           8
#define SERIAL_TELEMETRY_PARAM_PORT_E         0

// @pub 9;u32;3;r;packets;Counters: sent, complete RX handoffs (before DLM CRC), framing rejects
#define SERIAL_TELEMETRY_PARAM_PACKETS        9
#define SERIAL_TELEMETRY_PARAM_PACKETS_E      1

// @pub 10;f;3;r;speed;Packet rates per second for sent, received and rejected
#define SERIAL_TELEMETRY_PARAM_SPEED          10
#define SERIAL_TELEMETRY_PARAM_SPEED_E        2

// @pub 11;u32;1;w;baud;Baud rate (default: 115200)
#define SERIAL_TELEMETRY_PARAM_BAUD           11
#define SERIAL_TELEMETRY_PARAM_BAUD_E         3


#define SERIAL_TELEMETRY_PARAM_ENTRIES        4

/*

Packet framing:

[0] = 0xFE marker (excluded from CRC)
[1...n] = complete mesh packet: 7-byte header, 1-48 payload bytes, CRC8.
Mesh CRC validation belongs to DroneLinkManager.
*/

#define SERIAL_TELEMETRY_START_OF_FRAME  0xFE

// strings
static const char SERIAL_TELEMETRY_STR_SERIAL_TELEMETRY[] PROGMEM = "SerialTelemetry";


// class
class SerialTelemetryModule:  public NetworkInterfaceModule {
protected:
  Stream *_port = nullptr;
  serialTransport::MeshDecoder _decoder;

  uint32_t _packetsReceived;
  uint32_t _packetsRejected;
  uint32_t _packetsSent;
  unsigned long _packetsTimer;


public:

  SerialTelemetryModule(uint8_t id, DroneSystem* ds);

  uint8_t getInterfaceType();

  virtual void setup();
  virtual void loop();

  void setPort(Stream *port);

  // network interface methods
  boolean sendPacket(uint8_t *buffer, DRONE_LINK_TRANSPORT_ADDRESS transportAddress);

};

#endif
