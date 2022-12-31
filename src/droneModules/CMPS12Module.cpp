#include "CMPS12Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include <LittleFS.h>


CMPS12Module::CMPS12Module(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
 {
   setTypeName(FPSTR(CMPS12_STR_CMPS12));
   _location[0] = -1.8;
   _location[1] = 52;

   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;  // 1 sec

   // subs
   initSubs(CMPS12_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[CMPS12_SUB_LOCATION_E];
   sub->addrParam = CMPS12_SUB_LOCATION_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, CMPS12_SUB_LOCATION);
   setParamName(FPSTR(STRING_LOCATION), &sub->param);

   // pubs
   initParams(CMPS12_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = CMPS12_I2C_ADDRESS;

   // init param entries
   _params[CMPS12_PARAM_HEADING_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, CMPS12_PARAM_HEADING);
   _params[CMPS12_PARAM_HEADING_E].name = FPSTR(STRING_HEADING);
   _params[CMPS12_PARAM_HEADING_E].nameLen = sizeof(STRING_HEADING);
   _params[CMPS12_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[CMPS12_PARAM_DECLINATION_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, CMPS12_PARAM_DECLINATION);
   _params[CMPS12_PARAM_DECLINATION_E].name = FPSTR(STRING_DECLINATION);
   _params[CMPS12_PARAM_DECLINATION_E].nameLen = sizeof(STRING_DECLINATION);
   _params[CMPS12_PARAM_DECLINATION_E].data.f[0] = 0;
   _params[CMPS12_PARAM_DECLINATION_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[CMPS12_PARAM_TRIM_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, CMPS12_PARAM_TRIM);
   _params[CMPS12_PARAM_TRIM_E].name = FPSTR(STRING_TRIM);
   _params[CMPS12_PARAM_TRIM_E].nameLen = sizeof(STRING_TRIM);
   _params[CMPS12_PARAM_TRIM_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[CMPS12_PARAM_TRIM_E].data.f[0] = 0;

   _params[CMPS12_PARAM_VECTOR_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, CMPS12_PARAM_VECTOR);
   _params[CMPS12_PARAM_VECTOR_E].name = FPSTR(STRING_VECTOR);
   _params[CMPS12_PARAM_VECTOR_E].nameLen = sizeof(STRING_VECTOR);
   _params[CMPS12_PARAM_VECTOR_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   _params[CMPS12_PARAM_VECTOR_E].data.f[0] = 0;
   _params[CMPS12_PARAM_VECTOR_E].data.f[1] = 0;
}


DEM_NAMESPACE* CMPS12Module::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(CMPS12_STR_CMPS12,0,true);
}

void CMPS12Module::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {

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

  //dem->registerCommand(ns, STRING_TRIM, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}

void CMPS12Module::doReset() {
  Log.noticeln("[CMPS.dR]");
  I2CBaseModule::doReset();
/*
  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  if (_sensor) {
    setError( _sensor->begin() ? 0 : 1 );
    if (_error) {
      Log.errorln(CMPS12_STR_CMPS12);
    }
  }*/
  Log.noticeln("[CMPS.dR] end");
}


void CMPS12Module::setup() {
  I2CBaseModule::setup();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  // test to see if sensor responds on correct address:
  if (!DroneWire::scanAddress(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0])) {
    Log.errorln("[CMPS12] Module not detected on I2C bus");
    setError(1);
    disable();
    return;
  }
}

void CMPS12Module::update() {
  if (!_setupDone) return;

  // called when a param is updated by handleLinkMessage

  // see if location has changed
  int newLon = round(_subs[CMPS12_SUB_LOCATION_E].param.data.f[0]);
  int newLat = round(_subs[CMPS12_SUB_LOCATION_E].param.data.f[1]);

  if (newLon != _location[0] || newLat != _location[1]) {
    _location[0] = newLon;
    _location[1] = newLat;

    // read declination value from mag.dat file
    /*
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
        _params[CMPS12_PARAM_DECLINATION_E].data.f[0] = decl;
      }

      file.close();
    }
    */
  }
}

void CMPS12Module::loop() {
  I2CBaseModule::loop();

  //Log.noticeln("CMPS12.loop");

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  Wire.beginTransmission(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]);  //starts communication with CMPS12
  Wire.write(CMPS12_ANGLE_8);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 5 bytes from the CMPS12
  // this will give us the 8 bit bearing, 
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0], 5);       
  
  while(Wire.available() < 5);        // Wait for all bytes to come back
  
  // Read back the 5 bytes
  uint8_t angle8 = Wire.read(); // 0..255        
  uint8_t high_byte = Wire.read();
  uint8_t low_byte = Wire.read();
  int8_t pitch = Wire.read();   // +-90
  int8_t roll = Wire.read();    // +-90
  
  uint16_t angle16 = (high_byte << 8) | low_byte;  // Calculate 16 bit angle, 0.. 3599

  float headingDegrees = angle16 / 10.0f;

  float declinationAngle = _params[CMPS12_PARAM_DECLINATION_E].data.f[0]; 
  headingDegrees += declinationAngle;

  // wrap to 0..360
  if (headingDegrees > 360) headingDegrees -= 360;
  if (headingDegrees < 0) headingDegrees += 360;

  // add trim
  headingDegrees += _params[CMPS12_PARAM_TRIM_E].data.f[0];

  // wrap to 0..360
  if (headingDegrees > 360) headingDegrees -= 360;
  if (headingDegrees < 0) headingDegrees += 360;

  _params[CMPS12_PARAM_HEADING_E].data.f[0] = headingDegrees;

  _params[CMPS12_PARAM_VECTOR_E].data.f[0] = pitch;
  _params[CMPS12_PARAM_VECTOR_E].data.f[1] = roll;

  // publish param entries
  publishParamEntries();

}

