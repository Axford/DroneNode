#include "NMEAModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../pinConfig.h"

NMEAModule::NMEAModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(NMEA_STR_NMEA));

   _numParamEntries = NMEA_PARAM_ENTRIES;
   _params = new DRONE_PARAM_ENTRY[_numParamEntries];

   _loopInterval = 1000;

   _portNum = 1;
   _baud = 38400;

   _nmea = new MicroNMEA(_buffer, sizeof(_buffer));

   // defaults
   for (uint8_t i=0; i<_numParamEntries; i++) {
     _params[i].publish = false;
   }

   //nmea.setUnknownSentenceHandler(printUnknownSentence);
   _params[NMEA_PARAM_LOCATION_E].param = NMEA_PARAM_LOCATION;
   _params[NMEA_PARAM_LOCATION_E].name = FPSTR(DRONE_STR_LOCATION);
   _params[NMEA_PARAM_LOCATION_E].nameLen = sizeof(DRONE_STR_LOCATION);
   _params[NMEA_PARAM_LOCATION_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[NMEA_PARAM_LOCATION_E].data.f[0] = 0;
   _params[NMEA_PARAM_LOCATION_E].data.f[1] = 0;
   _params[NMEA_PARAM_LOCATION_E].data.f[2] = 0;

   _params[NMEA_PARAM_SATELLITES_E].param = NMEA_PARAM_SATELLITES;
   _params[NMEA_PARAM_SATELLITES_E].name = FPSTR(DRONE_STR_SATELLITES);
   _params[NMEA_PARAM_SATELLITES_E].nameLen = sizeof(DRONE_STR_SATELLITES);
   _params[NMEA_PARAM_SATELLITES_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[NMEA_PARAM_SATELLITES_E].data.uint8[0] = 0;

   _params[NMEA_PARAM_HEADING_E].param = NMEA_PARAM_HEADING;
   _params[NMEA_PARAM_HEADING_E].name = FPSTR(DRONE_STR_HEADING);
   _params[NMEA_PARAM_HEADING_E].nameLen = sizeof(DRONE_STR_HEADING);
   _params[NMEA_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[NMEA_PARAM_HEADING_E].data.f[0] = 0;

   _params[NMEA_PARAM_SPEED_E].param = NMEA_PARAM_SPEED;
   _params[NMEA_PARAM_SPEED_E].name = FPSTR(DRONE_STR_SPEED);
   _params[NMEA_PARAM_SPEED_E].nameLen = sizeof(DRONE_STR_SPEED);
   _params[NMEA_PARAM_SPEED_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[NMEA_PARAM_SPEED_E].data.f[0] = 0;

   _params[NMEA_PARAM_HDOP_E].param = NMEA_PARAM_HDOP;
   _params[NMEA_PARAM_HDOP_E].name = FPSTR(DRONE_STR_HDOP);
   _params[NMEA_PARAM_HDOP_E].nameLen = sizeof(DRONE_STR_HDOP);
   _params[NMEA_PARAM_HDOP_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[NMEA_PARAM_HDOP_E].data.uint8[0] = 0;
}


void NMEAModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

  _portNum = obj[DRONE_STR_PORT] | _portNum;
  switch(_portNum) {
    case 0: setPort(&Serial); break;
    case 1: setPort(&Serial1); break;
    case 2: setPort(&Serial2); break;
  }

  _baud = obj[DRONE_STR_BAUD] | _baud;
}

void NMEAModule::setup() {
  DroneModule::setup();

  switch(_portNum) {
    //case 0: Serial.begin(_baud, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX); break;
    case 1: Serial1.begin(_baud, SERIAL_8N1, PIN_SERIAL1_RX, PIN_SERIAL1_TX); break;
    case 2: Serial2.begin(_baud, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX); break;
  }
}


void NMEAModule::loop() {
  DroneModule::loop();

  while (_port->available()) {
    char c = _port->read();
    //Serial.print(c);
    if (_nmea->process( c )) {

      if (_nmea->isUnknown()) {
        // pass to AIS decoder
        //Log.noticeln(F("Unknown NMEA Message: "));
        //Log.noticeln(_nmea->getSentence());
        //getSentence

        // TODO:
      } else {
        if (_nmea->isValid()) {
          //Log.noticeln(F("Fresh GPS"));
          _params[NMEA_PARAM_LOCATION_E].data.f[1] = _nmea->getLatitude() / 1000000.;
          _params[NMEA_PARAM_LOCATION_E].data.f[0] = _nmea->getLongitude() / 1000000.;
          long alt = 0;
          _nmea->getAltitude(alt);
          _params[NMEA_PARAM_LOCATION_E].data.f[2] = alt/1000.0;

          _params[NMEA_PARAM_SATELLITES_E].data.uint8[0] = _nmea->getNumSatellites();

          float v = _nmea->getSpeed() / 1000.0;
          if (v >= 0) {
            _params[NMEA_PARAM_SPEED_E].data.f[0] = v;
          }

          v = _nmea->getCourse() / 1000.0;
          if (v >= 0) {
            _params[NMEA_PARAM_HEADING_E].data.f[0] = v;
          }

          _params[NMEA_PARAM_HDOP_E].data.uint8[0] = _nmea->getHDOP();

          publishParamEntries();
        }
      }

    }
  }
}

void NMEAModule::setPort(Stream *port) {
  _port = port;
}
