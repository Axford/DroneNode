/*

Publishes received messages
Re-transmits messages it has been subscribed to
Handles packet framing, sync, etc

*/
#ifndef TELEMETRY_MODULE_H
#define TELEMETRY_MODULE_H

#include "Arduino.h"
#include "../DroneModule.h"
#include "../DroneLinkMsg.h"
#include <FastCRC.h>

#include "../pinConfig.h"

/*

Packet framing:

byte    = value
[0]     = stand of frame marker = 0xFD
[1...n] = DroneLinkMsg raw data
[n+1]   = CRC8 of DroneLinkMsg
*/

#define TELEMETRY_START_OF_FRAME  0xFE

// strings
static const char TELEMETRY_STR_TELEMETRY[] PROGMEM = "Telemetry";


// class
class TelemetryModule:  public DroneModule {
protected:
  uint8_t _portNum;
  Stream *_port;
  uint32_t _baud;
  FastCRC8 _CRC8;
  uint8_t _buffer[sizeof(DRONE_LINK_MSG)+2];
  DroneLinkMsg _receivedMsg;
  uint8_t _decodeState;
  /*
  0 = waiting for start
  1 = found start, waiting to confirm length
  2 = reading payload
  3 = checking CRC
  */
  uint8_t _receivedSize;

public:

  TelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm);

  void loadConfiguration(JsonObject &obj);

  virtual void handleLinkMessage(DroneLinkMsg *msg);

  virtual void setup();
  virtual void loop();

  void setPort(Stream *port);

};

#endif
