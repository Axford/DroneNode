#include "QMC5883LModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include <SPIFFS.h>


QMC5883LModule::QMC5883LModule(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
 {
   setTypeName(FPSTR(QMC5883L_STR_QMC5883L));
   //_params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = QMC5883L_I2C_ADDRESS;
   _sensor = NULL;
   _location[0] = -1.8;
   _location[1] = 52;

   // subs
   initSubs(QMC5883L_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[QMC5883L_SUB_LOCATION_E];
   sub->addrParam = QMC5883L_SUB_LOCATION_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_SUB_LOCATION);
   setParamName(FPSTR(STRING_LOCATION), &sub->param);

   // pubs
   initParams(QMC5883L_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = QMC5883L_I2C_ADDRESS;

   // init param entries
   _params[QMC5883L_PARAM_VECTOR_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_PARAM_VECTOR);
   _params[QMC5883L_PARAM_VECTOR_E].name = FPSTR(STRING_VECTOR);
   _params[QMC5883L_PARAM_VECTOR_E].nameLen = sizeof(STRING_VECTOR);
   _params[QMC5883L_PARAM_VECTOR_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);

   _params[QMC5883L_PARAM_HEADING_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, QMC5883L_PARAM_HEADING);
   _params[QMC5883L_PARAM_HEADING_E].name = FPSTR(STRING_HEADING);
   _params[QMC5883L_PARAM_HEADING_E].nameLen = sizeof(STRING_HEADING);
   _params[QMC5883L_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[QMC5883L_PARAM_DECLINATION_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_PARAM_DECLINATION);
   _params[QMC5883L_PARAM_DECLINATION_E].name = FPSTR(STRING_DECLINATION);
   _params[QMC5883L_PARAM_DECLINATION_E].nameLen = sizeof(STRING_DECLINATION);
   _params[QMC5883L_PARAM_DECLINATION_E].data.f[0] = 0;
   _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[QMC5883L_PARAM_CALIB_X_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_PARAM_CALIB_X);
   _params[QMC5883L_PARAM_CALIB_X_E].name = FPSTR(STRING_CALIB_X);
   _params[QMC5883L_PARAM_CALIB_X_E].nameLen = sizeof(STRING_CALIB_X);
   _params[QMC5883L_PARAM_CALIB_X_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[QMC5883L_PARAM_CALIB_X_E].data.f[0] = -1;
   _params[QMC5883L_PARAM_CALIB_X_E].data.f[1] = 0;
   _params[QMC5883L_PARAM_CALIB_X_E].data.f[2] = 1;

   _params[QMC5883L_PARAM_CALIB_Y_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_PARAM_CALIB_Y);
   _params[QMC5883L_PARAM_CALIB_Y_E].name = FPSTR(STRING_CALIB_Y);
   _params[QMC5883L_PARAM_CALIB_Y_E].nameLen = sizeof(STRING_CALIB_Y);
   _params[QMC5883L_PARAM_CALIB_Y_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[QMC5883L_PARAM_CALIB_Y_E].data.f[0] = -1;
   _params[QMC5883L_PARAM_CALIB_Y_E].data.f[1] = 0;
   _params[QMC5883L_PARAM_CALIB_Y_E].data.f[2] = 1;

   _params[QMC5883L_PARAM_TRIM_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_PARAM_TRIM);
   _params[QMC5883L_PARAM_TRIM_E].name = FPSTR(STRING_TRIM);
   _params[QMC5883L_PARAM_TRIM_E].nameLen = sizeof(STRING_TRIM);
   _params[QMC5883L_PARAM_TRIM_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[QMC5883L_PARAM_TRIM_E].data.f[0] = 0;

   _params[QMC5883L_PARAM_LIMITS_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_PARAM_LIMITS);
   _params[QMC5883L_PARAM_LIMITS_E].name = FPSTR(STRING_LIMITS);
   _params[QMC5883L_PARAM_LIMITS_E].nameLen = sizeof(STRING_LIMITS);
   _params[QMC5883L_PARAM_LIMITS_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 16);
   _params[QMC5883L_PARAM_LIMITS_E].data.f[0] = 0;
   _params[QMC5883L_PARAM_LIMITS_E].data.f[1] = 0;
   _params[QMC5883L_PARAM_LIMITS_E].data.f[2] = 0;
   _params[QMC5883L_PARAM_LIMITS_E].data.f[3] = 0;

   _params[QMC5883L_PARAM_SAMPLES_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_PARAM_SAMPLES);
   _params[QMC5883L_PARAM_SAMPLES_E].name = FPSTR(STRING_SAMPLES);
   _params[QMC5883L_PARAM_SAMPLES_E].nameLen = sizeof(STRING_SAMPLES);
   _params[QMC5883L_PARAM_SAMPLES_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 16);
   _params[QMC5883L_PARAM_SAMPLES_E].data.uint32[0] = 0;
   _params[QMC5883L_PARAM_SAMPLES_E].data.uint32[1] = 0;
   _params[QMC5883L_PARAM_SAMPLES_E].data.uint32[2] = 0;
   _params[QMC5883L_PARAM_SAMPLES_E].data.uint32[3] = 0;
}

QMC5883LModule::~QMC5883LModule() {
  if (_sensor) delete _sensor;
}


void QMC5883LModule::doReset() {
  Log.noticeln("[HMC.dR]");
  I2CBaseModule::doReset();
/*
  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  if (_sensor) {
    setError( _sensor->begin() ? 0 : 1 );
    if (_error) {
      Log.errorln(QMC5883L_STR_QMC5883L);
    }
  }*/
  Log.noticeln("[HMC.dR] end");
}


void QMC5883LModule::setup() {
  I2CBaseModule::setup();

  if (!_sensor) {
    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
    /*_sensor = new Adafruit_QMC5883_Unified(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]);
    if (!_sensor->begin() ){
      Log.errorln("Failed to init QMC5883L");
    }*/
    _sensor = new QMC5883LCompass();
    _sensor->init();

    // initialise limits to match calibration limits - clockwise from North
    // y+
    _params[QMC5883L_PARAM_LIMITS_E].data.f[0] = _params[QMC5883L_PARAM_CALIB_Y_E].data.f[2];
    // x+
    _params[QMC5883L_PARAM_LIMITS_E].data.f[1] = _params[QMC5883L_PARAM_CALIB_X_E].data.f[2];
    // y-
    _params[QMC5883L_PARAM_LIMITS_E].data.f[2] = _params[QMC5883L_PARAM_CALIB_Y_E].data.f[0];
    // x-
    _params[QMC5883L_PARAM_LIMITS_E].data.f[3] = _params[QMC5883L_PARAM_CALIB_X_E].data.f[0];
  }
}

void QMC5883LModule::update() {
  if (!_setupDone) return;

  // called when a param is updated by handleLinkMessage

  // see if location has changed
  int newLon = round(_subs[QMC5883L_SUB_LOCATION_E].param.data.f[0]);
  int newLat = round(_subs[QMC5883L_SUB_LOCATION_E].param.data.f[1]);

  if (newLon != _location[0] || newLat != _location[1]) {
    _location[0] = newLon;
    _location[1] = newLat;

    // read declination value from mag.dat file
    if (SPIFFS.exists(F("/mag.dat"))) {
      File file = SPIFFS.open(F("/mag.dat"), FILE_READ);

      int minLon = -90;
      int maxLon = 0;
      int minLat = 0;
      //int maxLat = 60;

      int lonPoints = maxLon - minLon + 1;

      int mapIndex = (newLon - minLon) + (newLat - minLat) * lonPoints;

      if (file.seek(mapIndex)) {
        float decl = (file.read()-128) / 4.0f;
        _params[QMC5883L_PARAM_DECLINATION_E].data.f[0] = decl;
      }

      file.close();
    }
  }
}

void QMC5883LModule::loop() {
  I2CBaseModule::loop();

  //Log.noticeln("QMC5883L.loop");

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  // get sensor values
  /*
  sensors_event_t event;
  _sensor->getEvent(&event);
  _sensor->getEvent(&event); // get twice?


  _params[QMC5883L_PARAM_VECTOR_E].data.f[0] = event.magnetic.x;
  _params[QMC5883L_PARAM_VECTOR_E].data.f[1] = event.magnetic.y;
  _params[QMC5883L_PARAM_VECTOR_E].data.f[2] = event.magnetic.z;
  */

  _sensor->read();

  _params[QMC5883L_PARAM_VECTOR_E].data.f[0] = _sensor->getX() / 100.0;
  _params[QMC5883L_PARAM_VECTOR_E].data.f[1] = _sensor->getY() / 100.0;
  _params[QMC5883L_PARAM_VECTOR_E].data.f[2] = _sensor->getZ() / 100.0;

  // update calibration
  // X
  _params[QMC5883L_PARAM_CALIB_X_E].data.f[0] = min(_params[QMC5883L_PARAM_CALIB_X_E].data.f[0], _params[QMC5883L_PARAM_VECTOR_E].data.f[0]);
  _params[QMC5883L_PARAM_CALIB_X_E].data.f[2] = max(_params[QMC5883L_PARAM_CALIB_X_E].data.f[2], _params[QMC5883L_PARAM_VECTOR_E].data.f[0]);

  //_params[QMC5883L_PARAM_CALIB_X_E].data.f[1] = (_params[QMC5883L_PARAM_CALIB_X_E].data.f[0] + _params[QMC5883L_PARAM_CALIB_X_E].data.f[2])/2;
  _params[QMC5883L_PARAM_CALIB_X_E].data.f[1] = (_params[QMC5883L_PARAM_LIMITS_E].data.f[1] + _params[QMC5883L_PARAM_LIMITS_E].data.f[3]) / 2;

  // Y
  _params[QMC5883L_PARAM_CALIB_Y_E].data.f[0] = min(_params[QMC5883L_PARAM_CALIB_Y_E].data.f[0], _params[QMC5883L_PARAM_VECTOR_E].data.f[1]);
  _params[QMC5883L_PARAM_CALIB_Y_E].data.f[2] = max(_params[QMC5883L_PARAM_CALIB_Y_E].data.f[2], _params[QMC5883L_PARAM_VECTOR_E].data.f[1]);

  //_params[QMC5883L_PARAM_CALIB_Y_E].data.f[1] = (_params[QMC5883L_PARAM_CALIB_Y_E].data.f[0] + _params[QMC5883L_PARAM_CALIB_Y_E].data.f[2])/2;
  _params[QMC5883L_PARAM_CALIB_Y_E].data.f[1] = (_params[QMC5883L_PARAM_LIMITS_E].data.f[0] + _params[QMC5883L_PARAM_LIMITS_E].data.f[2]) / 2;


  float heading = atan2(_params[QMC5883L_PARAM_VECTOR_E].data.f[1] - _params[QMC5883L_PARAM_CALIB_Y_E].data.f[1],
                        _params[QMC5883L_PARAM_VECTOR_E].data.f[0] - _params[QMC5883L_PARAM_CALIB_X_E].data.f[1]);

  // rotate by -90 deg to account for sensor mounting orientation with y+ forward
  heading -= PI/2;

  float declinationAngle = _params[QMC5883L_PARAM_DECLINATION_E].data.f[0] * PI / 180.0f; // convert to radians
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = (heading * 180.0f / PI);

  // wrap to 0..360
  if (headingDegrees > 360) headingDegrees -= 360;
  if (headingDegrees < 0) headingDegrees += 360;

  // update limits when within +-20 degrees of quadrant
  // y+
  if (headingDegrees > -20 && headingDegrees < 20) {
    updateQuadrant(0, _params[QMC5883L_PARAM_VECTOR_E].data.f[1]);
  }

  // y-
  if (headingDegrees > 160 && headingDegrees < 200) {
    updateQuadrant(2, _params[QMC5883L_PARAM_VECTOR_E].data.f[1]);
  }

  // x+
  if (headingDegrees > 70 && headingDegrees < 110) {
    updateQuadrant(3, _params[QMC5883L_PARAM_VECTOR_E].data.f[0]);
  }

  // x-
  if (headingDegrees > 250 && headingDegrees < 290) {
    updateQuadrant(1, _params[QMC5883L_PARAM_VECTOR_E].data.f[0]);
  }

  // add trim
  headingDegrees += _params[QMC5883L_PARAM_TRIM_E].data.f[0];

  // wrap to 0..360
  if (headingDegrees > 360) headingDegrees -= 360;
  if (headingDegrees < 0) headingDegrees += 360;

  _params[QMC5883L_PARAM_HEADING_E].data.f[0] = headingDegrees;

/*
  Serial.print("Mag: ");
  Serial.print(_params[QMC5883L_PARAM_VECTOR_E].data.f[0]);
  Serial.print(", ");
  Serial.print(_params[QMC5883L_PARAM_VECTOR_E].data.f[1]);
  Serial.print(" = ");
  Serial.println(heading);
*/

  // error check
  /*
  if (isnan(_params[QMC5883L_PARAM_SHUNTV_E].data.f)) {
    setError(1);  // will be cleared by next watchdog
  }
  */

  // publish param entries
  publishParamEntries();

}


void QMC5883LModule::updateQuadrant(uint8_t quadrant, float v) {
  float avgWindow = 20;

  // start applying noise rejection after sufficient samples received
  if (_params[QMC5883L_PARAM_SAMPLES_E].data.uint32[quadrant] > 100) {
    float ratio = v / _params[QMC5883L_PARAM_LIMITS_E].data.f[quadrant];
    if (ratio < 0.8 || ratio > 1.2) return;
  }

  _params[QMC5883L_PARAM_LIMITS_E].data.f[quadrant] =
    (_params[QMC5883L_PARAM_LIMITS_E].data.f[quadrant] * (avgWindow-1) +  v
  ) / avgWindow;

  _params[QMC5883L_PARAM_SAMPLES_E].data.uint32[quadrant]++;
}
