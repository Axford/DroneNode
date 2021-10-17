/*

Publishes received messages
Re-transmits messages it has been subscribed to
Handles packet framing, sync, etc

*/
#ifndef UDP_TELEMETRY_MODULE_H
#define UDP_TELEMETRY_MODULE_H

#include "Arduino.h"
#include "../DroneModule.h"
#include "../DroneLinkMsg.h"
#include <WiFiUdp.h>
/*

Packet framing:

byte    = value
[0...n] = DroneLinkMsg raw data
*/

#define UDP_TELEMETRY_PORT   8007

static const char UDP_TELEMETRY_STR_UDP_TELEMETRY[] PROGMEM = "UDPTelemetry";


class UDPTelemetryModule:  public DroneModule {
protected:
  WiFiUDP _udp;
  uint16_t _port;
  uint8_t _broadcast[4];
  uint8_t _rBuffer[sizeof(DRONE_LINK_MSG)];
  DroneLinkMsg _receivedMsg;

  uint8_t _receivedSize;

public:

  UDPTelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);

  void loadConfiguration(JsonObject &obj);

  virtual void handleLinkMessage(DroneLinkMsg *msg);

  virtual void setup();
  virtual void loop();

};

#endif
