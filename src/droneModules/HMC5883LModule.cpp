#include "HMC5883LModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include <SPIFFS.h>


HMC5883LModule::HMC5883LModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  I2CBaseModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(HMC5883L_STR_HMC5883L));
   _addr = HMC5883L_I2C_ADDRESS;

   _location[0] = 0;
   _location[1] = 0;

   _loopInterval = 1000;

   // subs
   initSubs(HMC5883L_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[HMC5883L_SUB_LOCATION_E];
   sub->addrParam = HMC5883L_SUB_LOCATION_ADDR;
   sub->param.param = HMC5883L_SUB_LOCATION;
   setParamName(FPSTR(STRING_LOCATION), &sub->param);

   // pubs
   initParams(HMC5883L_PARAM_ENTRIES);

   // init param entries
   _params[HMC5883L_PARAM_VECTOR_E].param = HMC5883L_PARAM_VECTOR;
   _params[HMC5883L_PARAM_VECTOR_E].name = FPSTR(STRING_VECTOR);
   _params[HMC5883L_PARAM_VECTOR_E].nameLen = sizeof(STRING_VECTOR);
   _params[HMC5883L_PARAM_VECTOR_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);

   _params[HMC5883L_PARAM_HEADING_E].param = HMC5883L_PARAM_HEADING;
   _params[HMC5883L_PARAM_HEADING_E].name = FPSTR(STRING_HEADING);
   _params[HMC5883L_PARAM_HEADING_E].nameLen = sizeof(STRING_HEADING);
   _params[HMC5883L_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[HMC5883L_PARAM_DECLINATION_E].param = HMC5883L_PARAM_DECLINATION;
   _params[HMC5883L_PARAM_DECLINATION_E].name = FPSTR(STRING_DECLINATION);
   _params[HMC5883L_PARAM_DECLINATION_E].nameLen = sizeof(STRING_DECLINATION);
   _params[HMC5883L_PARAM_DECLINATION_E].data.f[0] = 0;

   _params[HMC5883L_PARAM_CALIB_X_E].param = HMC5883L_PARAM_CALIB_X;
   _params[HMC5883L_PARAM_CALIB_X_E].name = FPSTR(STRING_CALIB_X);
   _params[HMC5883L_PARAM_CALIB_X_E].nameLen = sizeof(STRING_CALIB_X);
   _params[HMC5883L_PARAM_CALIB_X_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);

   _params[HMC5883L_PARAM_CALIB_Y_E].param = HMC5883L_PARAM_CALIB_Y;
   _params[HMC5883L_PARAM_CALIB_Y_E].name = FPSTR(STRING_CALIB_Y);
   _params[HMC5883L_PARAM_CALIB_Y_E].nameLen = sizeof(STRING_CALIB_Y);
   _params[HMC5883L_PARAM_CALIB_Y_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);

}

HMC5883LModule::~HMC5883LModule() {
  if (_sensor) delete _sensor;
}


void HMC5883LModule::doReset() {
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_bus);

  setError( _sensor->begin() ? 0 : 1 );
  if (_error) {
    Log.errorln(HMC5883L_STR_HMC5883L);
  }
}


void HMC5883LModule::loadConfiguration(JsonObject &obj) {
  I2CBaseModule::loadConfiguration(obj);

  // instantiate sensor object, now _addr is known
  _sensor = new Adafruit_HMC5883_Unified(_id);

  // read default declination
  if (obj.containsKey(STRING_DECLINATION)) {
    _params[HMC5883L_PARAM_CALIB_X_E].data.f[0] = obj[STRING_DECLINATION];
  }

  // read calibration values
  if (obj.containsKey(STRING_CALIB_X)) {
    Log.noticeln(STRING_CALIB_X);
    JsonArray array = obj[STRING_CALIB_X].as<JsonArray>();
    if (array.size()==2) {
      _params[HMC5883L_PARAM_CALIB_X_E].data.f[0] = array[0];
      _params[HMC5883L_PARAM_CALIB_X_E].data.f[2] = array[1];
    }
  }

  if (obj.containsKey(STRING_CALIB_Y)) {
    Log.noticeln(STRING_CALIB_Y);
    JsonArray array = obj[STRING_CALIB_Y].as<JsonArray>();
    if (array.size()==2) {
      _params[HMC5883L_PARAM_CALIB_Y_E].data.f[0] = array[0];
      _params[HMC5883L_PARAM_CALIB_Y_E].data.f[2] = array[1];
    }
  }
}


void HMC5883LModule::update() {
  // see if location has changed
  int newLon = round(_subs[HMC5883L_SUB_LOCATION_E].param.data.f[0]);
  int newLat = round(_subs[HMC5883L_SUB_LOCATION_E].param.data.f[1]);

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
        _params[HMC5883L_PARAM_DECLINATION_E].data.f[0] = decl;
      }

      file.close();
    }
  }
}

void HMC5883LModule::loop() {
  I2CBaseModule::loop();

  //Log.noticeln("HMC5883L.loop");

  DroneWire::selectChannel(_bus);


  // get sensor values
  sensors_event_t event;
  _sensor->getEvent(&event);

  _params[HMC5883L_PARAM_VECTOR_E].data.f[0] = event.magnetic.x;
  _params[HMC5883L_PARAM_VECTOR_E].data.f[1] = event.magnetic.y;
  _params[HMC5883L_PARAM_VECTOR_E].data.f[2] = event.magnetic.z;

  // update calibration
  // X
  _params[HMC5883L_PARAM_CALIB_X_E].data.f[0] = min(_params[HMC5883L_PARAM_CALIB_X_E].data.f[0], _params[HMC5883L_PARAM_VECTOR_E].data.f[0]);
  _params[HMC5883L_PARAM_CALIB_X_E].data.f[2] = max(_params[HMC5883L_PARAM_CALIB_X_E].data.f[2], _params[HMC5883L_PARAM_VECTOR_E].data.f[0]);
  _params[HMC5883L_PARAM_CALIB_X_E].data.f[1] = (_params[HMC5883L_PARAM_CALIB_X_E].data.f[0] + _params[HMC5883L_PARAM_CALIB_X_E].data.f[2])/2;

  // Y
  _params[HMC5883L_PARAM_CALIB_Y_E].data.f[0] = min(_params[HMC5883L_PARAM_CALIB_Y_E].data.f[0], _params[HMC5883L_PARAM_VECTOR_E].data.f[1]);
  _params[HMC5883L_PARAM_CALIB_Y_E].data.f[2] = max(_params[HMC5883L_PARAM_CALIB_Y_E].data.f[2], _params[HMC5883L_PARAM_VECTOR_E].data.f[1]);
  _params[HMC5883L_PARAM_CALIB_Y_E].data.f[1] = (_params[HMC5883L_PARAM_CALIB_Y_E].data.f[0] + _params[HMC5883L_PARAM_CALIB_Y_E].data.f[2])/2;


  float heading = atan2(event.magnetic.y - _params[HMC5883L_PARAM_CALIB_Y_E].data.f[1],
                        event.magnetic.x - _params[HMC5883L_PARAM_CALIB_X_E].data.f[1]);

  float declinationAngle = _params[HMC5883L_PARAM_DECLINATION_E].data.f[0] * PI / 180.0f; // convert to radians
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180.0f / PI;

  _params[HMC5883L_PARAM_HEADING_E].data.f[0] = headingDegrees;


  // error check
  /*
  if (isnan(_params[HMC5883L_PARAM_SHUNTV_E].data.f)) {
    setError(1);  // will be cleared by next watchdog
  }
  */

  // publish param entries
  publishParamEntries();

}
