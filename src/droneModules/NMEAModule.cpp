#include "NMEAModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../pinConfig.h"
#include "strings.h"
#include "../navMath.h"
#include "OLEDTomThumbFont.h"

NMEAModule::NMEAModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(NMEA_STR_NMEA));

   _port = NULL;
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   _nmea = new MicroNMEA(_buffer, sizeof(_buffer));
   //nmea.setUnknownSentenceHandler(printUnknownSentence);

   _historyCount = 0;

   // subs
   initSubs(NMEA_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[NMEA_SUB_CORRECTION_E];
   sub->addrParam = NMEA_SUB_CORRECTION_ADDR;
   sub->param.param = NMEA_SUB_CORRECTION;
   setParamName(FPSTR(STRING_CORRECTION), &sub->param);
   sub->param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 16);
   sub->param.data.f[0] = 0;
   sub->param.data.f[1] = 0;
   sub->param.data.f[2] = 0;
   sub->param.data.f[3] = 0;

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

   param = &_params[NMEA_PARAM_FIX_E];
   param->param = NMEA_PARAM_FIX;
   setParamName(FPSTR(STRING_FIX), param);
   _params[NMEA_PARAM_FIX_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[NMEA_PARAM_FIX_E].data.f[0] = 0;
   _params[NMEA_PARAM_FIX_E].data.f[1] = 0;
   _params[NMEA_PARAM_FIX_E].data.f[2] = 0;

   param = &_params[NMEA_PARAM_FOLLOWME_E];
   param->param = NMEA_PARAM_FOLLOWME;
   setParamName(FPSTR(STRING_FOLLOWME), param);
   _params[NMEA_PARAM_FOLLOWME_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[NMEA_PARAM_FOLLOWME_E].data.f[0] = 0;
   _params[NMEA_PARAM_FOLLOWME_E].data.f[1] = 0;
   _params[NMEA_PARAM_FOLLOWME_E].data.f[2] = 5;
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
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_CORRECTION, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$correction"), DRONE_LINK_MSG_TYPE_FLOAT, pha);

  dem->registerCommand(ns, STRING_PORT, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_BAUD, DRONE_LINK_MSG_TYPE_UINT32_T, ph);
  dem->registerCommand(ns, STRING_FIX, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_FOLLOWME, DRONE_LINK_MSG_TYPE_FLOAT, ph);

}

void NMEAModule::setup() {
  DroneModule::setup();

  switch(_params[NMEA_PARAM_PORT_E].data.uint8[0]) {
    //case 0: Serial.begin(_baud, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX); break;
    case 1:
      Log.noticeln(F("[NMEA.s] Serial 1 at %u"), _params[NMEA_PARAM_BAUD_E].data.uint32[0]);
      Serial1.begin(_params[NMEA_PARAM_BAUD_E].data.uint32[0], SERIAL_8N1, PIN_SERIAL1_RX, PIN_SERIAL1_TX);
      setPort(&Serial1);
      break;
    case 2:
      Log.noticeln(F("[NMEA.s] Serial 2 at %u"), _params[NMEA_PARAM_BAUD_E].data.uint32[0]);
      Serial2.begin(_params[NMEA_PARAM_BAUD_E].data.uint32[0], SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX);
      setPort(&Serial2);
      break;
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
  //uint8_t cc = 0;  // to limit max chars read
  while (_port->available()) {
    char c = _port->read();
    //cc++;

    //Serial.print(c);
    if (_nmea->process( c )) {
      //Log.noticeln(F("[NMEA.l] c"));

      if (_nmea->isUnknown()) {
        // pass to AIS decoder
        //Log.noticeln(F("Unknown NMEA Message: "));
        //Log.noticeln(_nmea->getSentence());
        //getSentence

        // TODO:
      } else {
        if (_nmea->isValid()) {
          //Log.noticeln(F("Fresh GPS"));

          float tempf[3];
          tempf[1] = _nmea->getLatitude() / 1000000.;
          tempf[0] = _nmea->getLongitude() / 1000000.;
          long alt = 0;
          _nmea->getAltitude(alt);
          tempf[2] = alt/1000.0;

          // do we have a known (fix) location
          if (_params[NMEA_PARAM_FIX_E].data.f[0] != 0) {
            // generate a correction factor
            float correction[4];
            correction[0] = _params[NMEA_PARAM_FIX_E].data.f[0] - tempf[0];
            correction[1] = _params[NMEA_PARAM_FIX_E].data.f[1] - tempf[1];
            correction[2] = _params[NMEA_PARAM_FIX_E].data.f[2] - tempf[2];

            updateAndPublishParam(&_subs[NMEA_SUB_CORRECTION_E].param, (uint8_t*)&correction, sizeof(correction));

            // calculate magnitude of correction in meters
            correction[3] = calculateDistanceBetweenCoordinates(
              _params[NMEA_PARAM_FIX_E].data.f[0],
              _params[NMEA_PARAM_FIX_E].data.f[1],
              tempf[0],
              tempf[1]
            );

            // now overwrite location to fixed location
            tempf[0] = _params[NMEA_PARAM_FIX_E].data.f[0];
            tempf[1] = _params[NMEA_PARAM_FIX_E].data.f[1];
            tempf[2] = _params[NMEA_PARAM_FIX_E].data.f[2];

          } else {
            // do we have a differential correction to apply?
            tempf[0] += _subs[NMEA_SUB_CORRECTION_E].param.data.f[0];
            tempf[1] += _subs[NMEA_SUB_CORRECTION_E].param.data.f[1];
            tempf[2] += _subs[NMEA_SUB_CORRECTION_E].param.data.f[2];
          }


          if (tempf[0] != 0 && tempf[1] != 0) {

            // accumulate in history buffer
            // shuffle back
            for (uint8_t i=0; i<NMEA_HISTORY_DEPTH-1; i++) {
              _history[i][0] = _history[i+1][0];
              _history[i][1] = _history[i+1][1];
              _history[i][2] = _history[i+1][2];
            }
            // add to head
            _history[NMEA_HISTORY_DEPTH-1][0] = tempf[0];
            _history[NMEA_HISTORY_DEPTH-1][1] = tempf[1];
            _history[NMEA_HISTORY_DEPTH-1][2] = millis();
            // see if buffer full
            if (_historyCount < NMEA_HISTORY_DEPTH) {
              _historyCount++;
            } else {
              // calc and publish speed/heading
              float t = (_history[NMEA_HISTORY_DEPTH-1][2] - _history[0][2])/1000;
              float d = calculateDistanceBetweenCoordinates(
                _history[NMEA_HISTORY_DEPTH-1][0],
                _history[NMEA_HISTORY_DEPTH-1][1],
                _history[0][0],
                _history[0][1]
              );
              float speed = d / t;
              updateAndPublishParam(&_params[NMEA_PARAM_SPEED_E], (uint8_t*)&speed, 4);
            }


            updateAndPublishParam(&_params[NMEA_PARAM_LOCATION_E], (uint8_t*)&tempf, sizeof(tempf));

            // update followMe
            tempf[2] = _params[NMEA_PARAM_FOLLOWME_E].data.f[2];
            updateAndPublishParam(&_params[NMEA_PARAM_FOLLOWME_E], (uint8_t*)&tempf, sizeof(tempf));
          }

          uint8_t temp8 =  _nmea->getNumSatellites();
          updateAndPublishParam(&_params[NMEA_PARAM_SATELLITES_E], (uint8_t*)&temp8, sizeof(temp8));

          /*
          float v = _nmea->getSpeed() / 1000.0;
          if (v >= 0) {
            updateAndPublishParam(&_params[NMEA_PARAM_SPEED_E], (uint8_t*)&v, 4);
          }

          v = _nmea->getCourse() / 1000.0;
          if (v >= 0) {
            updateAndPublishParam(&_params[NMEA_PARAM_HEADING_E], (uint8_t*)&v, 4);
          }
          */


          temp8 = _nmea->getHDOP();
          updateAndPublishParam(&_params[NMEA_PARAM_HDOP_E], (uint8_t*)&temp8, sizeof(temp8));
        } else {
          //Log.errorln(F("Invalid NMEA sentence"));
        }
      }

    }
  }
  //Log.noticeln(F("[NMEA.l] end"));
}

void NMEAModule::setPort(Stream *port) {
  _port = port;
}


uint8_t NMEAModule::diagnosticDisplays() {
  return 1;
}

void NMEAModule::drawDiagnosticDisplay(SSD1306Wire *display, uint8_t page) {
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  // number of satellites
  display->setFont(TomThumb4x6);
  display->drawString(0, 17+4, "# Sat.");
  display->setFont(ArialMT_Plain_10);

  display->drawString(32, 17, String(_params[NMEA_PARAM_SATELLITES_E].data.uint8[0]));



}
