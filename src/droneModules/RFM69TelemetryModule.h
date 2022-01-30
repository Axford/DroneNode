/*

@type RFM69Telemetry
@description Manages DroneLink telemetry using an RFM69HW radio module

@config >>>
RFM69Telemetry.new 3
  name "RFM69"
  status 1  // enable
  .sub [@>0.0]  // subscribe to and transmit all traffic on this node
  .publish "RSSI"
.done
<<<

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

// @pub 8;f;1;RSSI;RSSI of received packets
#define RFM69_TELEMETRY_PARAM_RSSI           8
#define RFM69_TELEMETRY_PARAM_RSSI_E         0

// @pub 9;u32;3;packets;Packet counters for sent, received and rejected
#define RFM69_TELEMETRY_PARAM_PACKETS        9
#define RFM69_TELEMETRY_PARAM_PACKETS_E      1

// @pub 10;f;3;speed;Packet rates per second for sent, received and rejected
#define RFM69_TELEMETRY_PARAM_SPEED          10
#define RFM69_TELEMETRY_PARAM_SPEED_E        2

#define RFM69_TELEMETRY_PARAM_ENTRIES        3


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
  uint32_t _packetsSent;
  unsigned long _packetsTimer;
  FastCRC8 _CRC8;
  uint8_t _encryptKey[16];
  uint8_t _buffer[RH_RF69_MAX_MESSAGE_LEN];
public:

  RFM69TelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  virtual void handleLinkMessage(DroneLinkMsg *msg);

  virtual void setup();
  virtual void loop();

};

#endif
