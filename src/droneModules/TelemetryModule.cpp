#include "TelemetryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"


TelemetryModule::TelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(TELEMETRY_STR_TELEMETRY));
   _buffer[0] = TELEMETRY_START_OF_FRAME;
   _decodeState = 0;
   _receivedSize = 0;
   _portNum = 2;
   _baud = 57600;

   _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, sizeof(TELEMETRY_STR_TELEMETRY));
   strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, TELEMETRY_STR_TELEMETRY, sizeof(TELEMETRY_STR_TELEMETRY));
}

void TelemetryModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

  _portNum = obj[DRONE_STR_PORT] | _portNum;
  switch(_portNum) {
    case 0: setPort(&Serial); break;
    case 1: setPort(&Serial1); break;
    case 2: setPort(&Serial2); break;
  }

  _baud = obj[DRONE_STR_BAUD] | _baud;
}

void TelemetryModule::handleLinkMessage(DroneLinkMsg *msg) {
  DroneModule::handleLinkMessage(msg);

  if (!_enabled) return;

  // check to see if this is the same as the last message we received!
  // if so, we're getting stuck in a loop and the message should be ignored
  if (_receivedMsg.sameSignature(msg)) return;



  // ignore query messages
  if (msg->type() == DRONE_LINK_MSG_TYPE_QUERY ||
      msg->type() == DRONE_LINK_MSG_TYPE_NAMEQUERY) return;

  // package up and re-transmit over telemetry link
  // length is payload length + start byte + node + chan + param + paramtypelength + crc
  uint8_t transmitLength = msg->length() + sizeof(DRONE_LINK_ADDR) + 2;

  memcpy(_buffer + 1, &msg->_msg, transmitLength);

  // CRC excludes start byte and crc, so payload length + 4, or transmitlength - 2
  _buffer[transmitLength-1] = _CRC8.smbus(_buffer + 1, msg->length() + sizeof(DRONE_LINK_ADDR));

  _port->write(_buffer, transmitLength);

/*
  Serial.print(TELEMETRY_STR_TELEMETRY);

  for (uint8_t i=0; i<transmitLength; i++) {
    Serial.print(' ');
    Serial.print(_buffer[i], HEX);
  }
  Serial.print('\n');
*/

}

void TelemetryModule::setup() {
  DroneModule::setup();

  switch(_portNum) {
    //case 0: Serial.begin(_baud, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX); break;
    case 1: Serial1.begin(_baud, SERIAL_8N1, PIN_SERIAL1_RX, PIN_SERIAL1_TX); break;
    case 2: Serial2.begin(_baud, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX); break;
  }
}

void TelemetryModule::loop() {
  DroneModule::loop();

  if (_port->available()) {
    //Serial.print("T: ");
    while (_port->available())
    {
      uint8_t b = _port->read();
      //Serial.print(b, HEX);

      switch(_decodeState) {
        case 0: // waiting for start
          if (b == TELEMETRY_START_OF_FRAME) {
            _decodeState = 1;
            _receivedSize = 0;
          }
          break;

        case 1: // found start, waiting to confirm payload length
          if (_receivedSize == 0) {
            _receivedMsg._msg.source = b;
          } else if (_receivedSize == 1) {
            _receivedMsg._msg.node = b;
          } else if (_receivedSize == 2) {
            _receivedMsg._msg.channel = b;
          } else if (_receivedSize == 3) {
            _receivedMsg._msg.param = b;
          } else if (_receivedSize == 4) {
            _receivedMsg._msg.paramTypeLength = b;
            _decodeState = 2;
          }
          _receivedSize++;
          break;

        case 2: // reading payload
          _receivedMsg._msg.payload.uint8[_receivedSize - sizeof(DRONE_LINK_ADDR)] = b;
          if (_receivedSize == _receivedMsg.length() + sizeof(DRONE_LINK_ADDR) - 1) {
            _decodeState =3;
          }
          _receivedSize++;
          break;

        case 3: // checking CRC
          uint8_t crc = _CRC8.smbus((uint8_t*)&_receivedMsg._msg, _receivedMsg.length() + sizeof(DRONE_LINK_ADDR));
          if (crc == b) {
            _dlm->publishPeer(_receivedMsg, 30, _id);
          } else {
            Log.errorln("Telem CRC fail %x", crc);
          }
          _decodeState = 0;
          break;
      }
    }
    //Serial.print('\n');
  }
}

void TelemetryModule::setPort(Stream *port) {
  _port = port;
}
