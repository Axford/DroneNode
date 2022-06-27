/*

@type SerialTelemetry
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
#include <FastCRC.h>

#include "../pinConfig.h"

// @pub 8;u8;1;port;Serial port (0-2, default: 1)
#define SERIAL_TELEMETRY_PARAM_PORT           8
#define SERIAL_TELEMETRY_PARAM_PORT_E         0

// @pub 9;u32;3;packets;Packet counters for sent, received and rejected
#define SERIAL_TELEMETRY_PARAM_PACKETS        9
#define SERIAL_TELEMETRY_PARAM_PACKETS_E      1

// @pub 10;f;3;speed;Packet rates per second for sent, received and rejected
#define SERIAL_TELEMETRY_PARAM_SPEED          10
#define SERIAL_TELEMETRY_PARAM_SPEED_E        2

// @pub 11;u32;1;baud;Baud rate (default: 115200)
#define SERIAL_TELEMETRY_PARAM_BAUD           11
#define SERIAL_TELEMETRY_PARAM_BAUD_E         3


#define SERIAL_TELEMETRY_PARAM_ENTRIES        4

/*

Packet framing:

byte    = value
[0]     = stand of frame marker = 0xFD
[1...n] = DroneLinkMsg raw data
[n+1]   = CRC8 of DroneLinkMsg
*/

#define SERIAL_TELEMETRY_START_OF_FRAME  0xFE

// strings
static const char SERIAL_TELEMETRY_STR_SERIAL_TELEMETRY[] PROGMEM = "SerialTelemetry";


// class
class SerialTelemetryModule:  public NetworkInterfaceModule {
protected:
  uint8_t _portNum;
  Stream *_port;
  uint32_t _baud;
  FastCRC8 _CRC8;
  uint8_t _msgLen;
  uint8_t _buffer[DRONE_MESH_MSG_MAX_PACKET_SIZE+2];
  uint8_t _decodeState;
  /*
  0 = waiting for start
  1 = found start, waiting to confirm length
  2 = reading payload
  3 = checking CRC
  */

  uint32_t _packetsReceived;
  uint32_t _packetsRejected;
  uint32_t _packetsSent;
  unsigned long _packetsTimer;

  uint8_t _receivedSize;

public:

  SerialTelemetryModule(uint8_t id, DroneSystem* ds);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  uint8_t getInterfaceType();

  virtual void setup();
  virtual void loop();

  void setPort(Stream *port);

  // network interface methods
  boolean sendPacket(uint8_t *buffer);

};

#endif
