#include "HMC5883LModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include <SPIFFS.h>


HMC5883LModule::HMC5883LModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  I2CBaseModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(HMC5883L_STR_HMC5883L));
   //_params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = HMC5883L_I2C_ADDRESS;
   _sensor = NULL;
   _location[0] = -1.8;
   _location[1] = 52;

   // subs
   initSubs(HMC5883L_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[HMC5883L_SUB_LOCATION_E];
   sub->addrParam = HMC5883L_SUB_LOCATION_ADDR;
   sub->param.param = HMC5883L_SUB_LOCATION;
   setParamName(FPSTR(STRING_LOCATION), &sub->param);

   // pubs
   initParams(HMC5883L_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = HMC5883L_I2C_ADDRESS;

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
   _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[HMC5883L_PARAM_CALIB_X_E].param = HMC5883L_PARAM_CALIB_X;
   _params[HMC5883L_PARAM_CALIB_X_E].name = FPSTR(STRING_CALIB_X);
   _params[HMC5883L_PARAM_CALIB_X_E].nameLen = sizeof(STRING_CALIB_X);
   _params[HMC5883L_PARAM_CALIB_X_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[HMC5883L_PARAM_CALIB_X_E].data.f[0] = -1;
   _params[HMC5883L_PARAM_CALIB_X_E].data.f[1] = 0;
   _params[HMC5883L_PARAM_CALIB_X_E].data.f[2] = 1;

   _params[HMC5883L_PARAM_CALIB_Y_E].param = HMC5883L_PARAM_CALIB_Y;
   _params[HMC5883L_PARAM_CALIB_Y_E].name = FPSTR(STRING_CALIB_Y);
   _params[HMC5883L_PARAM_CALIB_Y_E].nameLen = sizeof(STRING_CALIB_Y);
   _params[HMC5883L_PARAM_CALIB_Y_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[HMC5883L_PARAM_CALIB_Y_E].data.f[0] = -1;
   _params[HMC5883L_PARAM_CALIB_Y_E].data.f[1] = 0;
   _params[HMC5883L_PARAM_CALIB_Y_E].data.f[2] = 1;

   _params[HMC5883L_PARAM_TRIM_E].param = HMC5883L_PARAM_TRIM;
   _params[HMC5883L_PARAM_TRIM_E].name = FPSTR(STRING_TRIM);
   _params[HMC5883L_PARAM_TRIM_E].nameLen = sizeof(STRING_TRIM);
   _params[HMC5883L_PARAM_TRIM_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[HMC5883L_PARAM_TRIM_E].data.f[0] = 0;

}

HMC5883LModule::~HMC5883LModule() {
  if (_sensor) delete _sensor;
}


DEM_NAMESPACE* HMC5883LModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(HMC5883L_STR_HMC5883L,0,true);
}

void HMC5883LModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {

  I2CBaseModule::registerParams(ns, dem);

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_LOCATION, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$location"), DRONE_LINK_MSG_TYPE_FLOAT, pha);

  dem->registerCommand(ns, STRING_CALIB_X, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_CALIB_Y, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_TRIM, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}

void HMC5883LModule::doReset() {
  Log.noticeln("[HMC.dR]");
  I2CBaseModule::doReset();
/*
  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  if (_sensor) {
    setError( _sensor->begin() ? 0 : 1 );
    if (_error) {
      Log.errorln(HMC5883L_STR_HMC5883L);
    }
  }*/
  Log.noticeln("[HMC.dR] end");
}


void HMC5883LModule::setup() {
  I2CBaseModule::setup();

  if (!_sensor) {
    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
    /*_sensor = new Adafruit_HMC5883_Unified(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]);
    if (!_sensor->begin() ){
      Log.errorln("Failed to init HMC5883L");
    }*/
    _sensor = new HMC5883L();
    _sensor->initialize();
  }
}

void HMC5883LModule::update() {
  if (!_setupDone) return;

  // called when a param is updated by handleLinkMessage

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

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  // get sensor values
  /*
  sensors_event_t event;
  _sensor->getEvent(&event);
  _sensor->getEvent(&event); // get twice?


  _params[HMC5883L_PARAM_VECTOR_E].data.f[0] = event.magnetic.x;
  _params[HMC5883L_PARAM_VECTOR_E].data.f[1] = event.magnetic.y;
  _params[HMC5883L_PARAM_VECTOR_E].data.f[2] = event.magnetic.z;
  */

  int16_t mx, my, mz;
  _sensor->getHeading(&mx, &my, &mz);

  _params[HMC5883L_PARAM_VECTOR_E].data.f[0] = mx / 100.0;
  _params[HMC5883L_PARAM_VECTOR_E].data.f[1] = my / 100.0;
  _params[HMC5883L_PARAM_VECTOR_E].data.f[2] = mz / 100.0;

  // update calibration
  // X
  _params[HMC5883L_PARAM_CALIB_X_E].data.f[0] = min(_params[HMC5883L_PARAM_CALIB_X_E].data.f[0], _params[HMC5883L_PARAM_VECTOR_E].data.f[0]);
  _params[HMC5883L_PARAM_CALIB_X_E].data.f[2] = max(_params[HMC5883L_PARAM_CALIB_X_E].data.f[2], _params[HMC5883L_PARAM_VECTOR_E].data.f[0]);
  _params[HMC5883L_PARAM_CALIB_X_E].data.f[1] = (_params[HMC5883L_PARAM_CALIB_X_E].data.f[0] + _params[HMC5883L_PARAM_CALIB_X_E].data.f[2])/2;

  // Y
  _params[HMC5883L_PARAM_CALIB_Y_E].data.f[0] = min(_params[HMC5883L_PARAM_CALIB_Y_E].data.f[0], _params[HMC5883L_PARAM_VECTOR_E].data.f[1]);
  _params[HMC5883L_PARAM_CALIB_Y_E].data.f[2] = max(_params[HMC5883L_PARAM_CALIB_Y_E].data.f[2], _params[HMC5883L_PARAM_VECTOR_E].data.f[1]);
  _params[HMC5883L_PARAM_CALIB_Y_E].data.f[1] = (_params[HMC5883L_PARAM_CALIB_Y_E].data.f[0] + _params[HMC5883L_PARAM_CALIB_Y_E].data.f[2])/2;


  float heading = atan2(_params[HMC5883L_PARAM_VECTOR_E].data.f[1] - _params[HMC5883L_PARAM_CALIB_Y_E].data.f[1],
                        _params[HMC5883L_PARAM_VECTOR_E].data.f[0] - _params[HMC5883L_PARAM_CALIB_X_E].data.f[1]);

  // rotate by -90 def to account for sensor mounting orientation with y+ forward
  heading -= PI/2;

  float declinationAngle = _params[HMC5883L_PARAM_DECLINATION_E].data.f[0] * PI / 180.0f; // convert to radians
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = (heading * 180.0f / PI) + _params[HMC5883L_PARAM_TRIM_E].data.f[0];

  // wrap to 0..360
  if (headingDegrees > 360) headingDegrees -= 360;
  if (headingDegrees < 0) headingDegrees += 360;

  _params[HMC5883L_PARAM_HEADING_E].data.f[0] = headingDegrees;

/*
  Serial.print("Mag: ");
  Serial.print(_params[HMC5883L_PARAM_VECTOR_E].data.f[0]);
  Serial.print(", ");
  Serial.print(_params[HMC5883L_PARAM_VECTOR_E].data.f[1]);
  Serial.print(" = ");
  Serial.println(heading);
*/

  // error check
  /*
  if (isnan(_params[HMC5883L_PARAM_SHUNTV_E].data.f)) {
    setError(1);  // will be cleared by next watchdog
  }
  */

  // publish param entries
  publishParamEntries();

}
