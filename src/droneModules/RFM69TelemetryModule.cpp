#include "RFM69TelemetryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "WiFi.h"
#include "../pinConfig.h"

RFM69TelemetryModule::RFM69TelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(RFM69_TELEMETRY_STR_RFM69_TELEMETRY));

   _packetsReceived = 0;

   // pubs
   initParams(RFM69_TELEMETRY_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[RFM69_TELEMETRY_PARAM_RSSI_E];
   param->param = RFM69_TELEMETRY_PARAM_RSSI;
   setParamName(FPSTR(DRONE_STR_RSSI), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

}

void RFM69TelemetryModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

}

void RFM69TelemetryModule::handleLinkMessage(DroneLinkMsg *msg) {
  DroneModule::handleLinkMessage(msg);

  if (!_enabled) return;

  if (_error > 0) return;

  // check to see if this is the same as the last message we received!
  // if so, we're getting stuck in a loop and the message should be ignored
  if (_receivedMsg.sameSignature(msg)) return;

  //Serial.print("RFM69: Sending: ");
  //msg->print();

  // TODO: fix packet relay between nodes (mesh) and avoiding circular loops of packets jumping between nodes

  _radio.send(255, (uint8_t*)&msg->_msg, msg->length() + sizeof(DRONE_LINK_ADDR));
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
    if ((_radio.DATALEN >= sizeof(DRONE_LINK_ADDR)+1) && _radio.DATALEN < sizeof(DRONE_LINK_MSG)) {
      _packetsReceived++;

      memcpy(&_receivedMsg._msg, _radio.DATA, _radio.DATALEN);
      //Serial.print("RFM69 Receveived: ");
      //_receivedMsg.print();

      _dlm->publish(_receivedMsg);

      // update RSSI - moving average
      float v = _params[RFM69_TELEMETRY_PARAM_RSSI_E].data.f[0];
      v = (9*v + _radio.RSSI)/10;
      _params[RFM69_TELEMETRY_PARAM_RSSI_E].data.f[0] = v;

      // publish every 10 packets
      if (_packetsReceived % 10 == 0) {
        publishParamEntries();
      }
    }
  }
}
