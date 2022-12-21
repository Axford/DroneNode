#include "RFM69TelemetryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "WiFi.h"
#include "../pinConfig.h"
#include "strings.h"
#include "DroneSystem.h"

//#include "RFM69registers.h"

RFM69TelemetryModule::RFM69TelemetryModule(uint8_t id, DroneSystem* ds):
  NetworkInterfaceModule ( id, ds )
 {
   setTypeName(FPSTR(RFM69_TELEMETRY_STR_RFM69_TELEMETRY));
   _packetsReceived = 0;
   _packetsRejected = 0;
   _packetsSent = 0;
   _packetsTimer = 0;

   for (uint8_t i=0; i<sizeof(_encryptKey); i++) {
     _encryptKey[i] = i + 10;
   }

   _radio = NULL;

   // pubs
   initParams(RFM69_TELEMETRY_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[RFM69_TELEMETRY_PARAM_RSSI_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, RFM69_TELEMETRY_PARAM_RSSI);
   setParamName(FPSTR(STRING_RSSI), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[RFM69_TELEMETRY_PARAM_PACKETS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, RFM69_TELEMETRY_PARAM_PACKETS);
   setParamName(FPSTR(STRING_PACKETS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 12);
   param->data.uint32[0] = 0;
   param->data.uint32[1] = 0;
   param->data.uint32[2] = 0;

   param = &_params[RFM69_TELEMETRY_PARAM_SPEED_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, RFM69_TELEMETRY_PARAM_SPEED);
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[0] = 0;
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[RFM69_TELEMETRY_PARAM_POWER_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, RFM69_TELEMETRY_PARAM_POWER);
   setParamName(FPSTR(STRING_POWER), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   param->data.f[0] = 20;

   param = &_params[RFM69_TELEMETRY_PARAM_FREQUENCY_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, RFM69_TELEMETRY_PARAM_FREQUENCY);
   setParamName(FPSTR(STRING_FREQUENCY), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   param->data.uint32[0] = 915;
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

  dem->registerCommand(ns, STRING_POWER, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}


uint8_t RFM69TelemetryModule::getInterfaceType() {
  // to be overridden
  return DRONE_MESH_INTERFACE_TYPE_RFM69;
}


void RFM69TelemetryModule::onParamWrite(DRONE_PARAM_ENTRY *param) {
  NetworkInterfaceModule::onParamWrite(param);

  if (getDroneLinkMsgParam(param->paramPriority) == RFM69_TELEMETRY_PARAM_POWER) {
    int p = param->data.f[0];
    if (p > 20) p = 20;
    if (p < -14) p = -14;

    if (_radio) _radio->setTxPower(p, true);
  }
}


void RFM69TelemetryModule::setup() {
  NetworkInterfaceModule::setup();

  if (_mgmtParams[DRONE_MODULE_PARAM_STATUS_E].data.uint8[0] == 0) return;

  //pinMode(PIN_IN0_0, INPUT);

  // register CS and INT pins
  if (!_ds->requestPin(PIN_SD_6, DRONE_SYSTEM_PIN_CAP_OUTPUT, this) ||
      !_ds->requestPin(PIN_IN0_0, DRONE_SYSTEM_PIN_CAP_INPUT, this) ) {
    Log.errorln(F("[RFM.s] Pins unavailable"));
    setError(1);
    disable();
  }

  if (!_radio) {
    _spi.setPins(19, 23, 18);  // MISO, MOSI, CLK
    _radio = new RH_RF69(PIN_SD_6, PIN_IN0_0, _spi);   // CS, INT
  }

  if (!_radio->init()) {
    Log.errorln(F("Failed to init RFM69 radio"));
    setError(1);
  } else {

    if (!_radio->setFrequency(_params[RFM69_TELEMETRY_PARAM_FREQUENCY_E].data.uint32[0]))
      Log.errorln("setFrequency failed");


    // maximum POWWWWAAAAAA!!
    _radio->setTxPower(_params[RFM69_TELEMETRY_PARAM_POWER_E].data.f[0], true);
    //_radio->spyMode(true);
    _radio->setEncryptionKey(_encryptKey);

    // register network interface
    _dlm->registerInterface(this);

    _interfaceState = true;

    Log.noticeln(F("RFM69 initialised"));
  }
}


void RFM69TelemetryModule::loop() {
  NetworkInterfaceModule::loop();

  if (!_radio) return;

  //check if something was received (could be an interrupt from the radio)
  if (_radio->available())
  {
    uint8_t len = sizeof(_buffer);
    if (_radio->recv(_buffer, &len)) {

      boolean validPacket = true;

      // check size
      if (len < 8 || len > DRONE_MESH_MSG_MAX_PACKET_SIZE+2) {
        validPacket = false;
        Log.errorln("[RFM.rP] invalid size");
      }

      // check start byte
      if (validPacket && _buffer[0] != RFM69_START_OF_FRAME) {
        validPacket = false;
        Log.errorln("[RFM.rP] invalid start");
      }

      // check CRC
      if (validPacket) {
        uint8_t crc = _CRC8.smbus(_buffer, len-1);
        if (crc != _buffer[len-1]) {
          validPacket = false;
          Log.errorln("[RFM.rP] CRC fail");
        }
      }

      if (validPacket) {
        // check size matches header
        uint8_t headerLen = getDroneMeshMsgTotalSize(&_buffer[1]);
        if (headerLen != len-2) {
          validPacket = false;
          Log.errorln("[RFM.rP] size mismatch");
        }
      }

      if (validPacket) {
        // pass onto DLM
        // calc receive metric
        int16_t rssi = abs(constrain(_radio->lastRssi()/2, -100, 0));
        // packet loss is severe by -40db, so set the limit to 45
        uint8_t metric = map(min((int)rssi, 45), 0, 45, 1, 15);

        // rolling average rssi
        _params[RFM69_TELEMETRY_PARAM_RSSI_E].data.f[0] = (_params[RFM69_TELEMETRY_PARAM_RSSI_E].data.f[0] * 15 + rssi)/16;

        /*
        Log.noticeln("[RFM.l] recv %u bytes", len);
        for (uint8_t i=0; i<len; i++) {
          Serial.print("  ");
          Serial.print(_buffer[i], BIN);
        }
        Serial.println();
        */

        receivePacket(&_buffer[1], metric);
        _packetsReceived++;
      }


      if (!validPacket) {
        _packetsRejected++;
        Log.warningln("[RFM.rP] Rejected packet: %u", _packetsRejected);
      }


    } else {
      // receive failed
      Log.warningln("[RFM.l] Receive failed");
    }
  }


  // update and publish packet counters
  if (millis() > _packetsTimer + 5000) {

    uint32_t delta[3];
    delta[0] = _packetsSent - _params[RFM69_TELEMETRY_PARAM_PACKETS_E].data.uint32[0];
    delta[1] = _packetsReceived - _params[RFM69_TELEMETRY_PARAM_PACKETS_E].data.uint32[1];
    delta[2] = _packetsRejected - _params[RFM69_TELEMETRY_PARAM_PACKETS_E].data.uint32[2];


    _params[RFM69_TELEMETRY_PARAM_PACKETS_E].data.uint32[0] = _packetsSent;
    _params[RFM69_TELEMETRY_PARAM_PACKETS_E].data.uint32[1] = _packetsReceived;
    _params[RFM69_TELEMETRY_PARAM_PACKETS_E].data.uint32[2] = _packetsRejected;

    publishParamEntry(&_params[RFM69_TELEMETRY_PARAM_PACKETS_E]);

    float dur = (millis() - _packetsTimer)/1000.0;
    for (uint8_t i=0; i<3; i++) {
      _params[RFM69_TELEMETRY_PARAM_SPEED_E].data.f[i] = delta[i] / dur;
    }
    publishParamEntry(&_params[RFM69_TELEMETRY_PARAM_SPEED_E]);

    // RSSI
    publishParamEntry(&_params[RFM69_TELEMETRY_PARAM_RSSI_E]);

    _packetsTimer = millis();
  }
}


boolean RFM69TelemetryModule::sendPacket(uint8_t *buffer) {

  if (!_enabled) return false;

  // wrap the DroneMesh message in a start byte and end CRC
  uint8_t txSize = getDroneMeshMsgTotalSize(buffer) + 2;

  memcpy(_buffer + 1, buffer, txSize-2);
  _buffer[0] = RFM69_START_OF_FRAME; // ensure this is set, given we reuse the buffer
  _buffer[txSize-1] = _CRC8.smbus(_buffer, txSize - 1);

  /*
  Log.noticeln("[RFM.sP] sending %u bytes", txSize);
  for (uint8_t i=0; i<txSize; i++) {
    Serial.print("  ");
    Serial.print(_buffer[i], BIN);
  }
  Serial.println();
  */

  _radio->send(_buffer, txSize);
  _radio->waitPacketSent(100);

  _packetsSent++;

  return true;
}
