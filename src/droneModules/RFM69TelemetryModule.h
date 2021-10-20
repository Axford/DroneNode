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

#include <SPI.h>
#include <RH_RF69.h>
//#include <RFM69.h>
#include <RHSoftwareSPI.h>

#include <FastCRC.h>
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

#define RFM69_START_OF_FRAME          0xFE

class RFM69TelemetryModule:  public DroneModule {
protected:
  RH_RF69 *_radio;
  RHSoftwareSPI _spi;
  DroneLinkMsg _receivedMsg;
  uint32_t _packetsReceived;
  uint32_t _packetsRejected;
  FastCRC8 _CRC8;
  uint8_t _encryptKey[16];
  uint8_t _buffer[RH_RF69_MAX_MESSAGE_LEN];
public:

  RFM69TelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void loadConfiguration(JsonObject &obj);

  virtual void handleLinkMessage(DroneLinkMsg *msg);

  virtual void setup();
  virtual void loop();

};

#endif
