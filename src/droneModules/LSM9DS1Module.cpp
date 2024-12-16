#include "LSM9DS1Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include <SPIFFS.h>


LSM9DS1Module::LSM9DS1Module(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
 {
   setTypeName(FPSTR(LSM9DS1_STR_LSM9DS1));
   //_params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = LSM9DS1_I2C_ADDRESS;
   _sensor = NULL;
   _location[0] = -1.8;
   _location[1] = 52;

   // subs
   initSubs(LSM9DS1_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[LSM9DS1_SUB_LOCATION_E];
   sub->addrParam = LSM9DS1_SUB_LOCATION_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, LSM9DS1_SUB_LOCATION);
   setParamName(FPSTR(STRING_LOCATION), &sub->param);

   // pubs
   initParams(LSM9DS1_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = LSM9DS1_I2C_ADDRESS_1;

   // init param entries
   _params[LSM9DS1_PARAM_VECTOR_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, LSM9DS1_PARAM_VECTOR);
   _params[LSM9DS1_PARAM_VECTOR_E].name = FPSTR(STRING_VECTOR);
   _params[LSM9DS1_PARAM_VECTOR_E].nameLen = sizeof(STRING_VECTOR);
   _params[LSM9DS1_PARAM_VECTOR_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);

   _params[LSM9DS1_PARAM_HEADING_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, LSM9DS1_PARAM_HEADING);
   _params[LSM9DS1_PARAM_HEADING_E].name = FPSTR(STRING_HEADING);
   _params[LSM9DS1_PARAM_HEADING_E].nameLen = sizeof(STRING_HEADING);
   _params[LSM9DS1_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[LSM9DS1_PARAM_DECLINATION_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, LSM9DS1_PARAM_DECLINATION);
   _params[LSM9DS1_PARAM_DECLINATION_E].name = FPSTR(STRING_DECLINATION);
   _params[LSM9DS1_PARAM_DECLINATION_E].nameLen = sizeof(STRING_DECLINATION);
   _params[LSM9DS1_PARAM_DECLINATION_E].data.f[0] = 0;
   _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[LSM9DS1_PARAM_CALIB_X_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, LSM9DS1_PARAM_CALIB_X);
   _params[LSM9DS1_PARAM_CALIB_X_E].name = FPSTR(STRING_CALIB_X);
   _params[LSM9DS1_PARAM_CALIB_X_E].nameLen = sizeof(STRING_CALIB_X);
   _params[LSM9DS1_PARAM_CALIB_X_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[LSM9DS1_PARAM_CALIB_X_E].data.f[0] = -1;
   _params[LSM9DS1_PARAM_CALIB_X_E].data.f[1] = 0;
   _params[LSM9DS1_PARAM_CALIB_X_E].data.f[2] = 1;

   _params[LSM9DS1_PARAM_CALIB_Y_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, LSM9DS1_PARAM_CALIB_Y);
   _params[LSM9DS1_PARAM_CALIB_Y_E].name = FPSTR(STRING_CALIB_Y);
   _params[LSM9DS1_PARAM_CALIB_Y_E].nameLen = sizeof(STRING_CALIB_Y);
   _params[LSM9DS1_PARAM_CALIB_Y_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[LSM9DS1_PARAM_CALIB_Y_E].data.f[0] = -1;
   _params[LSM9DS1_PARAM_CALIB_Y_E].data.f[1] = 0;
   _params[LSM9DS1_PARAM_CALIB_Y_E].data.f[2] = 1;

   _params[LSM9DS1_PARAM_TRIM_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, LSM9DS1_PARAM_TRIM);
   _params[LSM9DS1_PARAM_TRIM_E].name = FPSTR(STRING_TRIM);
   _params[LSM9DS1_PARAM_TRIM_E].nameLen = sizeof(STRING_TRIM);
   _params[LSM9DS1_PARAM_TRIM_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[LSM9DS1_PARAM_TRIM_E].data.f[0] = 0;

   _params[LSM9DS1_PARAM_LIMITS_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, LSM9DS1_PARAM_LIMITS);
   _params[LSM9DS1_PARAM_LIMITS_E].name = FPSTR(STRING_LIMITS);
   _params[LSM9DS1_PARAM_LIMITS_E].nameLen = sizeof(STRING_LIMITS);
   _params[LSM9DS1_PARAM_LIMITS_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 16);
   _params[LSM9DS1_PARAM_LIMITS_E].data.f[0] = 0;
   _params[LSM9DS1_PARAM_LIMITS_E].data.f[1] = 0;
   _params[LSM9DS1_PARAM_LIMITS_E].data.f[2] = 0;
   _params[LSM9DS1_PARAM_LIMITS_E].data.f[3] = 0;

}

LSM9DS1Module::~LSM9DS1Module() {
  if (_sensor) delete _sensor;
}


void LSM9DS1Module::doReset() {
  Log.noticeln("[HMC.dR]");
  I2CBaseModule::doReset();
/*
  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  if (_sensor) {
    setError( _sensor->begin() ? 0 : 1 );
    if (_error) {
      Log.errorln(LSM9DS1_STR_LSM9DS1);
    }
  }*/
  Log.noticeln("[HMC.dR] end");
}


void LSM9DS1Module::setup() {
  I2CBaseModule::setup();

  if (!_sensor) {
    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
    /*_sensor = new Adafruit_HMC5883_Unified(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]);
    if (!_sensor->begin() ){
      Log.errorln("Failed to init LSM9DS1");
    }*/
    _sensor = new Adafruit_LSM9DS1();
    if (_sensor->begin()) {
      // 1.) Set the accelerometer range
      _sensor->setupAccel(_sensor->LSM9DS1_ACCELRANGE_2G);
      //_sensor->setupAccel(_sensor->LSM9DS1_ACCELRANGE_4G);
      //_sensor->setupAccel(_sensor->LSM9DS1_ACCELRANGE_8G);
      //_sensor->setupAccel(_sensor->LSM9DS1_ACCELRANGE_16G);

      // 2.) Set the magnetometer sensitivity
      _sensor->setupMag(_sensor->LSM9DS1_MAGGAIN_4GAUSS);
      //_sensor->setupMag(_sensor->LSM9DS1_MAGGAIN_8GAUSS);
      //_sensor->setupMag(_sensor->LSM9DS1_MAGGAIN_12GAUSS);
      //_sensor->setupMag(_sensor->LSM9DS1_MAGGAIN_16GAUSS);

      // 3.) Setup the gyroscope
      _sensor->setupGyro(_sensor->LSM9DS1_GYROSCALE_245DPS);
      //_sensor->setupGyro(_sensor->LSM9DS1_GYROSCALE_500DPS);
      //_sensor->setupGyro(_sensor->LSM9DS1_GYROSCALE_2000DPS);
    } else {
      setError(1);
    }

    // initialise limits to match calibration limits - clockwise from North
    // y+
    _params[LSM9DS1_PARAM_LIMITS_E].data.f[0] = _params[LSM9DS1_PARAM_CALIB_Y_E].data.f[2];
    // x+
    _params[LSM9DS1_PARAM_LIMITS_E].data.f[1] = _params[LSM9DS1_PARAM_CALIB_X_E].data.f[2];
    // y-
    _params[LSM9DS1_PARAM_LIMITS_E].data.f[2] = _params[LSM9DS1_PARAM_CALIB_Y_E].data.f[0];
    // x-
    _params[LSM9DS1_PARAM_LIMITS_E].data.f[3] = _params[LSM9DS1_PARAM_CALIB_X_E].data.f[0];
  }
}

void LSM9DS1Module::update() {
  if (!_setupDone) return;

  // called when a param is updated by handleLinkMessage

  // see if location has changed
  int newLon = round(_subs[LSM9DS1_SUB_LOCATION_E].param.data.f[0]);
  int newLat = round(_subs[LSM9DS1_SUB_LOCATION_E].param.data.f[1]);

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
        _params[LSM9DS1_PARAM_DECLINATION_E].data.f[0] = decl;
      }

      file.close();
    }
  }
}

void LSM9DS1Module::loop() {
  I2CBaseModule::loop();

  //Log.noticeln("LSM9DS1.loop");

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  _sensor->read();

  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;

  _sensor->getEvent(&a, &m, &g, &temp);

  /*

  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" uT");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" uT");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" uT");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" rad/s");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" rad/s");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" rad/s");

  */

  _params[LSM9DS1_PARAM_VECTOR_E].data.f[0] = m.magnetic.x;
  _params[LSM9DS1_PARAM_VECTOR_E].data.f[1] = m.magnetic.y;
  _params[LSM9DS1_PARAM_VECTOR_E].data.f[2] = m.magnetic.z;

  // update calibration
  // X
  _params[LSM9DS1_PARAM_CALIB_X_E].data.f[0] = min(_params[LSM9DS1_PARAM_CALIB_X_E].data.f[0], _params[LSM9DS1_PARAM_VECTOR_E].data.f[0]);
  _params[LSM9DS1_PARAM_CALIB_X_E].data.f[2] = max(_params[LSM9DS1_PARAM_CALIB_X_E].data.f[2], _params[LSM9DS1_PARAM_VECTOR_E].data.f[0]);

  //_params[LSM9DS1_PARAM_CALIB_X_E].data.f[1] = (_params[LSM9DS1_PARAM_CALIB_X_E].data.f[0] + _params[LSM9DS1_PARAM_CALIB_X_E].data.f[2])/2;
  _params[LSM9DS1_PARAM_CALIB_X_E].data.f[1] = (_params[LSM9DS1_PARAM_LIMITS_E].data.f[1] + _params[LSM9DS1_PARAM_LIMITS_E].data.f[3]) / 2;

  // Y
  _params[LSM9DS1_PARAM_CALIB_Y_E].data.f[0] = min(_params[LSM9DS1_PARAM_CALIB_Y_E].data.f[0], _params[LSM9DS1_PARAM_VECTOR_E].data.f[1]);
  _params[LSM9DS1_PARAM_CALIB_Y_E].data.f[2] = max(_params[LSM9DS1_PARAM_CALIB_Y_E].data.f[2], _params[LSM9DS1_PARAM_VECTOR_E].data.f[1]);

  //_params[LSM9DS1_PARAM_CALIB_Y_E].data.f[1] = (_params[LSM9DS1_PARAM_CALIB_Y_E].data.f[0] + _params[LSM9DS1_PARAM_CALIB_Y_E].data.f[2])/2;
  _params[LSM9DS1_PARAM_CALIB_Y_E].data.f[1] = (_params[LSM9DS1_PARAM_LIMITS_E].data.f[0] + _params[LSM9DS1_PARAM_LIMITS_E].data.f[2]) / 2;


  float heading = atan2(_params[LSM9DS1_PARAM_VECTOR_E].data.f[1] - _params[LSM9DS1_PARAM_CALIB_Y_E].data.f[1],
                        _params[LSM9DS1_PARAM_VECTOR_E].data.f[0] - _params[LSM9DS1_PARAM_CALIB_X_E].data.f[1]);

  // rotate by -90 deg to account for sensor mounting orientation with y+ forward
  heading -= PI/2;

  float declinationAngle = _params[LSM9DS1_PARAM_DECLINATION_E].data.f[0] * PI / 180.0f; // convert to radians
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
  float samples = 20;
  // y+
  if (headingDegrees > -20 && headingDegrees < 20) {
    _params[LSM9DS1_PARAM_LIMITS_E].data.f[0] =
      (_params[LSM9DS1_PARAM_LIMITS_E].data.f[0] * (samples-1) +
      _params[LSM9DS1_PARAM_VECTOR_E].data.f[1]
    ) / samples;
  }

  // y-
  if (headingDegrees > 160 && headingDegrees < 200) {
    _params[LSM9DS1_PARAM_LIMITS_E].data.f[2] =
      (_params[LSM9DS1_PARAM_LIMITS_E].data.f[2] * (samples-1) +
      _params[LSM9DS1_PARAM_VECTOR_E].data.f[1]
    ) / samples;
  }

  // x+
  if (headingDegrees > 70 && headingDegrees < 110) {
    _params[LSM9DS1_PARAM_LIMITS_E].data.f[3] =
      (_params[LSM9DS1_PARAM_LIMITS_E].data.f[3] * (samples-1) +
      _params[LSM9DS1_PARAM_VECTOR_E].data.f[0]
    ) / samples;
  }

  // x-
  if (headingDegrees > 250 && headingDegrees < 290) {
    _params[LSM9DS1_PARAM_LIMITS_E].data.f[1] =
      (_params[LSM9DS1_PARAM_LIMITS_E].data.f[1] * (samples-1) +
      _params[LSM9DS1_PARAM_VECTOR_E].data.f[0]
    ) / samples;
  }

  // add trim
  headingDegrees += _params[LSM9DS1_PARAM_TRIM_E].data.f[0];

  // wrap to 0..360
  if (headingDegrees > 360) headingDegrees -= 360;
  if (headingDegrees < 0) headingDegrees += 360;

  _params[LSM9DS1_PARAM_HEADING_E].data.f[0] = headingDegrees;

/*
  Serial.print("Mag: ");
  Serial.print(_params[LSM9DS1_PARAM_VECTOR_E].data.f[0]);
  Serial.print(", ");
  Serial.print(_params[LSM9DS1_PARAM_VECTOR_E].data.f[1]);
  Serial.print(" = ");
  Serial.println(heading);
*/

  // error check
  /*
  if (isnan(_params[LSM9DS1_PARAM_SHUNTV_E].data.f)) {
    setError(1);  // will be cleared by next watchdog
  }
  */

  // publish param entries
  publishParamEntries();

}
