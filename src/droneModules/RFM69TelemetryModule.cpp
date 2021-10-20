#include "RFM69TelemetryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "WiFi.h"
#include "../pinConfig.h"
#include "strings.h"
//#include "RFM69registers.h"

RFM69TelemetryModule::RFM69TelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem):
  DroneModule ( id, dmm, dlm, dem )
 {
   setTypeName(FPSTR(RFM69_TELEMETRY_STR_RFM69_TELEMETRY));
   _packetsReceived = 0;
   _packetsRejected = 0;
   for (uint8_t i=0; i<sizeof(_encryptKey); i++) {
     _encryptKey[i] = i + 10;
   }

   _radio = NULL;

   // pubs
   initParams(RFM69_TELEMETRY_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[RFM69_TELEMETRY_PARAM_RSSI_E];
   param->param = RFM69_TELEMETRY_PARAM_RSSI;
   setParamName(FPSTR(STRING_RSSI), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

}

DEM_NAMESPACE* RFM69TelemetryModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(RFM69_TELEMETRY_STR_RFM69_TELEMETRY,0,true);
}

void RFM69TelemetryModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);

  //dem->registerCommand(ns, STRING_LOCATION, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}


void RFM69TelemetryModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

}

void RFM69TelemetryModule::handleLinkMessage(DroneLinkMsg *msg) {
  DroneModule::handleLinkMessage(msg);

  if (!_radio) return;

  //Log.noticeln("[RFM.hLM] a");

  if (!_enabled || !_setupDone) return;

  if (_error > 0) return;

  // check to see if this is the same as the last message we received!
  // if so, we're getting stuck in a loop and the message should be ignored
  if (_receivedMsg.sameSignature(msg)) {
    //Serial.print("RFM69: Blocked: ");
    //msg->print();
    return;
  }

  //Log.noticeln("[RFM.hLM] b");

  // only send messages that originate on this node
  boolean sendPacket = (msg->source() == _dlm->node());
  if (!sendPacket) {
    // OR!
    // on a different interface
    sendPacket = _dlm->getSourceInterface(msg->source()) != _id;

    // OR!!
    // that are queries
    if (!sendPacket) {
      sendPacket = msg->type() > DRONE_LINK_MSG_TYPE_CHAR;
    }
  }

  //Log.noticeln("[RFM.hLM] c");

  if (sendPacket) {
    Serial.print("[RFM69.hLM] ");
    msg->print();

    // TODO - is this length right????
    uint8_t transmitLength = msg->length() + sizeof(DRONE_LINK_ADDR) + 2 + 1;

    memcpy(_buffer + 1, &msg->_msg, transmitLength-1);
    _buffer[0] = RFM69_START_OF_FRAME; // ensure this is set, given we reuse the buffer
    _buffer[transmitLength-1] = _CRC8.smbus(_buffer + 1, msg->length() + sizeof(DRONE_LINK_ADDR)+1);

    //Log.noticeln("[RFM.hLM] d");

    //_radio->send(255, (uint8_t*)_buffer, transmitLength);
    _radio->send(_buffer, transmitLength);
    //_radio->waitPacketSent();
    Serial.print("[RFM69.hLM] ok");
  } else {
    //Serial.print("RFM69: Filtered: ");
    //msg->print();
    //Log.noticeln("[RFM.hLM] e");
  }
}

void RFM69TelemetryModule::setup() {
  DroneModule::setup();

  if (_mgmtParams[DRONE_MODULE_PARAM_STATUS_E].data.uint8[0] == 0) return;

  //pinMode(PIN_IN0_0, INPUT);

  if (!_radio) {
    _radio = new RH_RF69(PIN_SD_6, PIN_IN0_0);   // CS, INT
  }


  //_radio->setIrq(PIN_IN0_0);
  if (!_radio->init()) {
    Log.errorln(F("Failed to init RFM69 radio"));
    setError(1);
  } else {

    if (!_radio->setFrequency(915.0))
      Serial.println("setFrequency failed");


    _radio->setTxPower(14, true);
    //_radio->spyMode(true);
    _radio->setEncryptionKey(_encryptKey);

    // enable whitening
    // ????

    Log.noticeln(F("RFM69 initialised"));
  }
}

void RFM69TelemetryModule::loop() {
  DroneModule::loop();

  if (!_radio) return;

  //check if something was received (could be an interrupt from the radio)
  if (_radio->available())
  {
    uint8_t len = sizeof(_buffer);
    if (_radio->recv(_buffer, &len)) {
      if ((len >= sizeof(DRONE_LINK_ADDR)+3) && len <= sizeof(DRONE_LINK_MSG)+2) {

        // copy out the msg data
        memcpy(&_receivedMsg._msg, &_buffer[1], len-2);

        // calc CRC on what we received
        uint8_t crc = _CRC8.smbus((uint8_t*)&_receivedMsg._msg, _receivedMsg.length() + sizeof(DRONE_LINK_ADDR)+1);

        // valid packet if CRC match and decoded length matches
        if ( (crc == _buffer[len-1])  &&
             (len == _receivedMsg.length() + sizeof(DRONE_LINK_ADDR) + 3) ){

          // valid packet
          _packetsReceived++;

          //Serial.print("RFM69 Receveived: ");
          //_receivedMsg.print();
          int16_t RSSI = -_radio->lastRssi();

          _dlm->publishPeer(_receivedMsg, RSSI, _id);

          // update RSSI - moving average
          float v = _params[RFM69_TELEMETRY_PARAM_RSSI_E].data.f[0];
          v = (9*v + RSSI)/10;
          _params[RFM69_TELEMETRY_PARAM_RSSI_E].data.f[0] = v;

          // publish every xx packets
          if (_packetsReceived % 40 == 0) {
            publishParamEntry(&_params[RFM69_TELEMETRY_PARAM_RSSI_E]);
          }

        } else {
          _packetsRejected++;
          Log.warningln("[RFM.l] Rejected packet: %u", _packetsRejected);
        }
      }


    } else {
      // receive failed
      Log.warningln("[RFM.l] Receive failed");
    }
  }
}
