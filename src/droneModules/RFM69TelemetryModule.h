/*

Publishes received messages
Re-transmits messages it has been subscribed to
Handles packet framing, sync, etc

*/
#ifndef RFM69_TELEMETRY_MODULE_H
#define RFM69_TELEMETRY_MODULE_H

#include "Arduino.h"
#include "../DroneModule.h"
#include "../DroneLinkMsg.h"
#include <RFM69.h>
#include <SPI.h>
/*

Packet framing:

byte    = value
[0...n] = DroneLinkMsg raw data
*/

#define RFM69_TELEMETRY_PARAM_RSSI           8
#define RFM69_TELEMETRY_PARAM_RSSI_E         0

#define RFM69_TELEMETRY_PARAM_ENTRIES        1


#define RFM69_TELEMETRY_NETWORKID     66  //the same on all nodes that talk to each other
#define RFM69_TELEMETRY_FREQUENCY     RF69_915MHZ
//#define ENCRYPTKEY    "droneLink" //exactly the same 16 characters/bytes on all nodes!

static const char RFM69_TELEMETRY_STR_RFM69_TELEMETRY[] PROGMEM = "RFM69Telemetry";

#define RFM69_TELEMTRY_ENCRYPTKEY     "abcd1234dcba4321"

class RFM69TelemetryModule:  public DroneModule {
protected:
  RFM69 _radio;
  DroneLinkMsg _receivedMsg;
  uint32_t _packetsReceived;
public:

  RFM69TelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm);

  void loadConfiguration(JsonObject &obj);

  virtual void handleLinkMessage(DroneLinkMsg *msg);

  virtual void setup();
  virtual void loop();

};

#endif
