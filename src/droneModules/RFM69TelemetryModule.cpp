#include "RFM69TelemetryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "WiFi.h"
#include "../pinConfig.h"

#include "RFM69registers.h"

RFM69TelemetryModule::RFM69TelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(RFM69_TELEMETRY_STR_RFM69_TELEMETRY));
   _buffer[0] = RFM69_START_OF_FRAME;
   _packetsReceived = 0;
   _packetsRejected = 0;

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
  if (_receivedMsg.sameSignature(msg)) {
    //Serial.print("RFM69: Blocked: ");
    //msg->print();
    return;
  }


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

  if (sendPacket) {
    if (msg->type() == DRONE_LINK_MSG_TYPE_FLOAT) {
      //Serial.print("RFM69: Sending: ");
      //msg->print();
    }

    uint8_t transmitLength = msg->length() + sizeof(DRONE_LINK_ADDR) + 2;

    memcpy(_buffer + 1, &msg->_msg, transmitLength);

    _buffer[transmitLength-1] = _CRC8.smbus(_buffer + 1, msg->length() + sizeof(DRONE_LINK_ADDR));

    _radio.send(255, (uint8_t*)_buffer, transmitLength);
  } else {
    //Serial.print("RFM69: Filtered: ");
    //msg->print();
  }
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
    _radio.encrypt(RFM69_TELEMTRY_ENCRYPTKEY);

    // enable whitening
    _radio.writeReg(REG_PACKETCONFIG1,
      RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_WHITENING | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF );

    Log.noticeln(F("RFM69 initialised"));
  }
}

void RFM69TelemetryModule::loop() {
  DroneModule::loop();

  //check if something was received (could be an interrupt from the radio)
  if (_radio.receiveDone())
  {
    if ((_radio.DATALEN >= sizeof(DRONE_LINK_ADDR)+2) && _radio.DATALEN <= sizeof(DRONE_LINK_MSG)+2) {

      // copy out the msg data
      memcpy(&_receivedMsg._msg, &_radio.DATA[1], _radio.DATALEN-2);

      // calc CRC on what we received
      uint8_t crc = _CRC8.smbus((uint8_t*)&_receivedMsg._msg, _receivedMsg.length() + sizeof(DRONE_LINK_ADDR));

      // valid packet if CRC match and decoded length matches
      if ( (crc == _radio.DATA[_radio.DATALEN-1])  &&
           (_radio.DATALEN == _receivedMsg.length() + sizeof(DRONE_LINK_ADDR) + 2) ){

        // valid packet
        _packetsReceived++;

        //Serial.print("RFM69 Receveived: ");
        //_receivedMsg.print();
        int16_t RSSI = -_radio.RSSI;

        _dlm->publishPeer(_receivedMsg, RSSI, _id);

        // update RSSI - moving average
        float v = _params[RFM69_TELEMETRY_PARAM_RSSI_E].data.f[0];
        v = (9*v + _radio.RSSI)/10;
        _params[RFM69_TELEMETRY_PARAM_RSSI_E].data.f[0] = v;

        // publish every xx packets
        if (_packetsReceived % 40 == 0) {
          publishParamEntry(&_params[RFM69_TELEMETRY_PARAM_RSSI_E]);
        }

      } else {
        _packetsRejected++;
        Log.warningln("RFM: Rejected packet: %u", _packetsRejected);
      }


    }
  }
}
