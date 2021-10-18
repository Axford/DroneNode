#include "NavModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

NavModule::NavModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem):
  DroneModule ( id, dmm, dlm, dem)
 {
   setTypeName(FPSTR(NAV_STR_NAV));

   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   _updateNeeded = true;

   // subs
   initSubs(NAV_SUBS);

   // defaults
   for (uint8_t i=0; i<NAV_SUBS; i++) {
     _subs[i].param.data.f[0] = 0;
     _subs[i].param.data.f[1] = 0;
     _subs[i].param.data.f[2] = 0;
     _subs[i].param.data.f[3] = 0;
   }

   _subs[NAV_SUB_LOCATION_E].addrParam = NAV_SUB_LOCATION_ADDR;
   _subs[NAV_SUB_LOCATION_E].param.param = NAV_SUB_LOCATION;
   _subs[NAV_SUB_LOCATION_E].param.name = FPSTR(STRING_LOCATION);
   _subs[NAV_SUB_LOCATION_E].param.nameLen = sizeof(STRING_LOCATION);
   _subs[NAV_SUB_LOCATION_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);

   _subs[NAV_SUB_TARGET_E].addrParam = NAV_SUB_TARGET_ADDR;
   _subs[NAV_SUB_TARGET_E].param.param = NAV_SUB_TARGET;
   _subs[NAV_SUB_TARGET_E].param.name = FPSTR(STRING_TARGET);
   _subs[NAV_SUB_TARGET_E].param.nameLen = sizeof(STRING_TARGET);
   _subs[NAV_SUB_TARGET_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 16);
   // target: lon lat radius duration

   // pubs
   initParams(NAV_PARAM_ENTRIES);

   _params[NAV_PARAM_HEADING_E].param = NAV_PARAM_HEADING;
   _params[NAV_PARAM_HEADING_E].name = FPSTR(STRING_HEADING);
   _params[NAV_PARAM_HEADING_E].nameLen = sizeof(STRING_HEADING);
   _params[NAV_PARAM_HEADING_E].publish = true;
   _params[NAV_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[NAV_PARAM_DISTANCE_E].param = NAV_PARAM_DISTANCE;
   _params[NAV_PARAM_DISTANCE_E].name = FPSTR(STRING_DISTANCE);
   _params[NAV_PARAM_DISTANCE_E].nameLen = sizeof(STRING_DISTANCE);
   _params[NAV_PARAM_DISTANCE_E].publish = true;
   _params[NAV_PARAM_DISTANCE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[NAV_PARAM_MODE_E].param = NAV_PARAM_MODE;
   _params[NAV_PARAM_MODE_E].name = FPSTR(STRING_MODE);
   _params[NAV_PARAM_MODE_E].nameLen = sizeof(STRING_MODE);
   _params[NAV_PARAM_MODE_E].publish = true;
   _params[NAV_PARAM_MODE_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[NAV_PARAM_MODE_E].data.uint8[0] = NAV_TO;
}

NavModule::~NavModule() {

}

DEM_NAMESPACE* NavModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(NAV_STR_NAV,0,true);
}

void NavModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_LOCATION, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$location"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_TARGET, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$target"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_HEADING, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_DISTANCE, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}



void NavModule::setup() {
  DroneModule::setup();
}

void NavModule::loop() {
  DroneModule::loop();

  if (_updateNeeded) {
      update();
  }
}



void NavModule::update() {
  if (!_setupDone) return;

  // formulae from: https://www.movable-type.co.uk/scripts/latlong.html

  /*
  const R = 6371e3; // metres
  const φ1 = lat1 * Math.PI/180; // φ, λ in radians
  const φ2 = lat2 * Math.PI/180;
  const Δφ = (lat2-lat1) * Math.PI/180;
  const Δλ = (lon2-lon1) * Math.PI/180;
  */

  float lat1 = _subs[NAV_SUB_LOCATION_E].param.data.f[1];
  float lon1 = _subs[NAV_SUB_LOCATION_E].param.data.f[0];

  float lat2 = _subs[NAV_SUB_TARGET_E].param.data.f[1];
  float lon2 = _subs[NAV_SUB_TARGET_E].param.data.f[0];


  float R = 6371e3; // metres
  float lat1r = lat1 * PI/180; // φ, λ in radians
  float lat2r = lat2 * PI/180;
  float lon1r = lon1 * PI/180; // φ, λ in radians
  float lon2r = lon2 * PI/180;
  //float dlat = (lat2-lat1) * PI/180;
  //float dlon = (lon2-lon1) * PI/180;

  /*
  const x = (λ2-λ1) * Math.cos((φ1+φ2)/2);
  const y = (φ2-φ1);
  const d = Math.sqrt(x*x + y*y) * R;
  */
  float x = (lon2r-lon1r) * cos((lat1r+lat2r)/2);
  float y = (lat2r-lat1r);
  float d = sqrt(x*x + y*y) * R;


  y = sin(lon2r-lon1r) * cos(lat2r);
  x = cos(lat1r)*sin(lat2r) -
      sin(lat1r)*cos(lat2r)*cos(lon2r-lon1r);
  float p = atan2(y, x);
  float h = fmod(p*180/PI + 360, 360); // in degrees

  _params[NAV_PARAM_HEADING_E].data.f[0] = h;
  _params[NAV_PARAM_DISTANCE_E].data.f[0] = d;

  // check to see if we've reached the waypoint
  // by comparing d (distance to go) to the target radius [2]
  if (d < _subs[NAV_SUB_TARGET_E].param.data.f[2]) {
    // TODO - we made it... now what?
  }

  _updateNeeded = false;

  publishParamEntries();
}
