#include "NMEAModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../pinConfig.h"
#include "strings.h"

NMEAModule::NMEAModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem):
  DroneModule ( id, dmm, dlm, dem )
 {
   setTypeName(FPSTR(NMEA_STR_NMEA));

   _port = NULL;
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   _nmea = new MicroNMEA(_buffer, sizeof(_buffer));
   //nmea.setUnknownSentenceHandler(printUnknownSentence);

   // pubs
   initParams(NMEA_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[NMEA_PARAM_LOCATION_E];
   param->param = NMEA_PARAM_LOCATION;
   setParamName(FPSTR(STRING_LOCATION), param);
   _params[NMEA_PARAM_LOCATION_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[NMEA_PARAM_LOCATION_E].data.f[0] = 0;
   _params[NMEA_PARAM_LOCATION_E].data.f[1] = 0;
   _params[NMEA_PARAM_LOCATION_E].data.f[2] = 0;

   param = &_params[NMEA_PARAM_SATELLITES_E];
   param->param = NMEA_PARAM_SATELLITES;
   setParamName(FPSTR(STRING_SATELLITES), param);
   _params[NMEA_PARAM_SATELLITES_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[NMEA_PARAM_SATELLITES_E].data.uint8[0] = 0;

   param = &_params[NMEA_PARAM_HEADING_E];
   param->param = NMEA_PARAM_HEADING;
   setParamName(FPSTR(STRING_HEADING), param);
   _params[NMEA_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[NMEA_PARAM_HEADING_E].data.f[0] = 0;

   param = &_params[NMEA_PARAM_SPEED_E];
   param->param = NMEA_PARAM_SPEED;
   setParamName(FPSTR(STRING_SPEED), param);
   _params[NMEA_PARAM_SPEED_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[NMEA_PARAM_SPEED_E].data.f[0] = 0;

   param = &_params[NMEA_PARAM_HDOP_E];
   param->param = NMEA_PARAM_HDOP;
   setParamName(FPSTR(STRING_HDOP), param);
   _params[NMEA_PARAM_HDOP_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[NMEA_PARAM_HDOP_E].data.uint8[0] = 0;

   param = &_params[NMEA_PARAM_PORT_E];
   param->param = NMEA_PARAM_PORT;
   setParamName(FPSTR(STRING_PORT), param);
   _params[NMEA_PARAM_PORT_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[NMEA_PARAM_PORT_E].data.uint8[0] = 1;

   param = &_params[NMEA_PARAM_BAUD_E];
   param->param = NMEA_PARAM_BAUD;
   setParamName(FPSTR(STRING_BAUD), param);
   _params[NMEA_PARAM_BAUD_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[NMEA_PARAM_BAUD_E].data.uint32[0] = 38400;
}


DEM_NAMESPACE* NMEAModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(NMEA_STR_NMEA,0,true);
}

void NMEAModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_PORT, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_BAUD, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}


void NMEAModule::onParamWrite(DRONE_PARAM_ENTRY *param) {
  if (param->param == NMEA_PARAM_PORT) {
    Log.noticeln(F("[NMEA.oPW] port: %u"), param->data.uint8[0]);
    switch(param->data.uint8[0]) {
      //case 0: setPort(&Serial); break;
      case 1: setPort(&Serial1); break;
      case 2: setPort(&Serial2); break;
      default:
        _port = NULL;
        Log.errorln(F("[NMEA.s] invalid port: %u"), _params[NMEA_PARAM_PORT_E].data.uint8[0]);
        setError(1);
    }
  }
}

void NMEAModule::setup() {
  DroneModule::setup();

  switch(_params[NMEA_PARAM_PORT_E].data.uint8[0]) {
    //case 0: Serial.begin(_baud, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX); break;
    case 1: Serial1.begin(_params[NMEA_PARAM_BAUD_E].data.uint32[0], SERIAL_8N1, PIN_SERIAL1_RX, PIN_SERIAL1_TX); break;
    case 2: Serial2.begin(_params[NMEA_PARAM_BAUD_E].data.uint32[0], SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX); break;
    default:
      _port = NULL;
      Log.errorln(F("[NMEA.s] invalid port: %u"), _params[NMEA_PARAM_PORT_E].data.uint8[0]);
      setError(1);
  }
}


void NMEAModule::loop() {
  //Log.noticeln(F("[NMEA.l] a"));
  DroneModule::loop();

  if (!_port) {
    Log.errorln("[NMEA.l] Undefined port");
    setError(1);
    return;
  }

  //Log.noticeln(F("[NMEA.l] b"));
  uint8_t cc = 0;  // to limit max chars read
  while (_port->available() && cc<20) {
    char c = _port->read();
    cc++;

    //Serial.print(c);
    if (_nmea->process( c )) {
      //Log.noticeln(F("[NMEA.l] c"));

      if (_nmea->isUnknown()) {
        // pass to AIS decoder
        Log.noticeln(F("Unknown NMEA Message: "));
        Log.noticeln(_nmea->getSentence());
        //getSentence

        // TODO:
      } else {
        if (_nmea->isValid()) {
          Log.noticeln(F("Fresh GPS"));

          float tempf[3];
          tempf[1] = _nmea->getLatitude() / 1000000.;
          tempf[0] = _nmea->getLongitude() / 1000000.;
          long alt = 0;
          _nmea->getAltitude(alt);
          tempf[2] = alt/1000.0;
          updateAndPublishParam(&_params[NMEA_PARAM_LOCATION_E], (uint8_t*)&tempf, sizeof(tempf));

          uint8_t temp8 =  _nmea->getNumSatellites();
          updateAndPublishParam(&_params[NMEA_PARAM_SATELLITES_E], (uint8_t*)&temp8, sizeof(temp8));


          float v = _nmea->getSpeed() / 1000.0;
          if (v >= 0) {
            updateAndPublishParam(&_params[NMEA_PARAM_SPEED_E], (uint8_t*)&v, 4);
          }

          v = _nmea->getCourse() / 1000.0;
          if (v >= 0) {
            updateAndPublishParam(&_params[NMEA_PARAM_HEADING_E], (uint8_t*)&v, 4);
          }

          temp8 = _nmea->getHDOP();
          updateAndPublishParam(&_params[NMEA_PARAM_HDOP_E], (uint8_t*)&temp8, sizeof(temp8));
        } else {
          Log.errorln(F("Invalid NMEA sentence"));
        }
      }

    }
  }
  //Log.noticeln(F("[NMEA.l] end"));
}

void NMEAModule::setPort(Stream *port) {
  _port = port;
}
