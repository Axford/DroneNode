#include "RFM69TelemetryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "WiFi.h"
#include "../pinConfig.h"

RFM69TelemetryModule::RFM69TelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(RFM69_TELEMETRY_STR_RFM69_TELEMETRY));

}

void RFM69TelemetryModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

}

void RFM69TelemetryModule::handleLinkMessage(DroneLinkMsg *msg) {
  DroneModule::handleLinkMessage(msg);

  if (_error > 0) return;

  // check to see if this is the same as the last message we received!
  // if so, we're getting stuck in a loop and the message should be ignored
  if (_receivedMsg.sameSignature(msg)) return;

  Serial.print("RFM69: Sending: ");
  msg->print();

  _radio.send(255, (uint8_t*)&msg->_msg, msg->length()+4);
}

void RFM69TelemetryModule::setup() {
  DroneModule::setup();

  pinMode(PIN_IN0_0, INPUT);
  _radio.setIrq(PIN_IN0_0);
  if (!_radio.initialize(RFM69_TELEMETRY_FREQUENCY, _dlm->node(), RFM69_TELEMETRY_NETWORKID)) {
    Log.errorln(F("Failed to init RFM69 radio"));
    setError(1);
  } else {
    _radio.setHighPower(); // for RFM69HW/HCW!
    _radio.spyMode(true);
    Log.errorln(F("RFM69 initialised"));
  }
}

void RFM69TelemetryModule::loop() {
  DroneModule::loop();

  //check if something was received (could be an interrupt from the radio)
  if (_radio.receiveDone())
  {
    //print message received to serial
    /*
    Serial.print('[');Serial.print(_radio.SENDERID);Serial.print("] ");
    Serial.print((char*)_radio.DATA);
    Serial.print("   [RX_RSSI:");Serial.print(_radio.RSSI);Serial.print("]");
    Serial.println();
    */

    //check if received message is 2 bytes long, and check if the message is specifically "Hi"
    //if (_radio.DATALEN==2 && _radio.DATA[0]=='H' && _radio.DATA[1]=='i')

    if (_radio.DATALEN >= 5 && _radio.DATALEN < sizeof(DRONE_LINK_MSG)) {
      memcpy(&_receivedMsg._msg, _radio.DATA, _radio.DATALEN);
      Serial.print("RFM69 Receveived: ");
      _receivedMsg.print();

      _dlm->publish(_receivedMsg);
    }
  }

  /*
  int packetSize = _udp.parsePacket();
  if (packetSize > 0 && packetSize <= sizeof(DRONE_LINK_MSG)) {
    //Log.noticeln("RFM69 Received: ");
    int len = _udp.read((uint8_t*)&_receivedMsg._msg, sizeof(DRONE_LINK_MSG));
    if (len >= 5) {
      //_receivedMsg.print();
      _dlm->publish(_receivedMsg);
    } else if (packetSize > 0) {
      // error - packet size mismatch
      Log.errorln(F("RFM69T: Packet size mismatch"));
    }
  } else if (packetSize > 0) {
    Log.noticeln("RFM69 Rec bytes: %d", packetSize);
  }
  */
}
