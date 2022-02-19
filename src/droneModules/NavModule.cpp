#include "NavModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../strings.h"
#include "../navMath.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

NavModule::NavModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs)
 {
   setTypeName(FPSTR(NAV_STR_NAV));

   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   _atTarget = false;

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
   _subs[NAV_SUB_LOCATION_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NAV_SUB_LOCATION);
   _subs[NAV_SUB_LOCATION_E].param.name = FPSTR(STRING_LOCATION);
   _subs[NAV_SUB_LOCATION_E].param.nameLen = sizeof(STRING_LOCATION);
   _subs[NAV_SUB_LOCATION_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);

   _subs[NAV_SUB_TARGET_E].addrParam = NAV_SUB_TARGET_ADDR;
   _subs[NAV_SUB_TARGET_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NAV_SUB_TARGET);
   _subs[NAV_SUB_TARGET_E].param.name = FPSTR(STRING_TARGET);
   _subs[NAV_SUB_TARGET_E].param.nameLen = sizeof(STRING_TARGET);
   _subs[NAV_SUB_TARGET_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 16);
   // target: lon lat radius duration

   _subs[NAV_SUB_WIND_E].addrParam = NAV_SUB_WIND_ADDR;
   _subs[NAV_SUB_WIND_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NAV_SUB_WIND);
   _subs[NAV_SUB_WIND_E].param.name = FPSTR(STRING_WIND);
   _subs[NAV_SUB_WIND_E].param.nameLen = sizeof(STRING_WIND);
   _subs[NAV_SUB_WIND_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   // pubs
   initParams(NAV_PARAM_ENTRIES);

   _params[NAV_PARAM_HEADING_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, NAV_PARAM_HEADING);
   _params[NAV_PARAM_HEADING_E].name = FPSTR(STRING_HEADING);
   _params[NAV_PARAM_HEADING_E].nameLen = sizeof(STRING_HEADING);
   _params[NAV_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[NAV_PARAM_DISTANCE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, NAV_PARAM_DISTANCE);
   _params[NAV_PARAM_DISTANCE_E].name = FPSTR(STRING_DISTANCE);
   _params[NAV_PARAM_DISTANCE_E].nameLen = sizeof(STRING_DISTANCE);
   _params[NAV_PARAM_DISTANCE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[NAV_PARAM_MODE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, NAV_PARAM_MODE);
   _params[NAV_PARAM_MODE_E].name = FPSTR(STRING_MODE);
   _params[NAV_PARAM_MODE_E].nameLen = sizeof(STRING_MODE);
   _params[NAV_PARAM_MODE_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[NAV_PARAM_MODE_E].data.uint8[0] = NAV_IDLE;

   _params[NAV_PARAM_LAST_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, NAV_PARAM_LAST);
   _params[NAV_PARAM_LAST_E].name = FPSTR(STRING_LAST);
   _params[NAV_PARAM_LAST_E].nameLen = sizeof(STRING_LAST);
   _params[NAV_PARAM_LAST_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[NAV_PARAM_LAST_E].data.f[0] = 0;
   _params[NAV_PARAM_LAST_E].data.f[1] = 0;
   _params[NAV_PARAM_LAST_E].data.f[2] = 0;

   _params[NAV_PARAM_HOME_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, NAV_PARAM_HOME);
   _params[NAV_PARAM_HOME_E].name = FPSTR(STRING_HOME);
   _params[NAV_PARAM_HOME_E].nameLen = sizeof(STRING_HOME);
   _params[NAV_PARAM_HOME_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   _params[NAV_PARAM_HOME_E].data.f[0] = 0;
   _params[NAV_PARAM_HOME_E].data.f[1] = 0;

   _params[NAV_PARAM_CROSSTRACK_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NAV_PARAM_CROSSTRACK);
   _params[NAV_PARAM_CROSSTRACK_E].name = FPSTR(STRING_CROSSTRACK);
   _params[NAV_PARAM_CROSSTRACK_E].nameLen = sizeof(STRING_CROSSTRACK);
   _params[NAV_PARAM_CROSSTRACK_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[NAV_PARAM_CROSSTRACK_E].data.f[0] = 0;

   _params[NAV_PARAM_CORRECTION_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NAV_PARAM_CORRECTION);
   _params[NAV_PARAM_CORRECTION_E].name = FPSTR(STRING_CORRECTION);
   _params[NAV_PARAM_CORRECTION_E].nameLen = sizeof(STRING_CORRECTION);
   _params[NAV_PARAM_CORRECTION_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[NAV_PARAM_CORRECTION_E].data.f[0] = 20;

   DRONE_PARAM_ENTRY *param;
   param = &_params[NAV_PARAM_CROSSWIND_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NAV_PARAM_CROSSWIND);
   setParamName(FPSTR(STRING_CROSSWIND), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[NAV_PARAM_CROSSWIND_E].data.f[0] = 0.5;

   param = &_params[NAV_PARAM_ADJ_HEADING_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, NAV_PARAM_ADJ_HEADING);
   setParamName(FPSTR(STRING_ADJ_HEADING), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
}

NavModule::~NavModule() {

}

DEM_NAMESPACE* NavModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(NAV_STR_NAV,0,true);
}

void NavModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_LOCATION, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$location"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_TARGET, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$target"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_WIND, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$wind"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_HEADING, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_DISTANCE, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_LAST, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_HOME, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_CROSSTRACK, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_CORRECTION, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_CROSSWIND, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_MODE, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
}



void NavModule::setup() {
  DroneModule::setup();

  // register private commands
  String nsName = "_" + String(getName());
  DEM_NAMESPACE* ns = _dem->createNamespace(nsName.c_str(),0,true);

  DEMCommandHandler gotoH = std::bind(&NavModule::nav_goto, this, _1, _2, _3, _4);
  _dem->registerCommand(ns, PSTR("goto"), DRONE_LINK_MSG_TYPE_FLOAT, gotoH);

  DEMCommandHandler inRadiusH = std::bind(&NavModule::nav_inRadius, this, _1, _2, _3, _4);
  _dem->registerCommand(ns, PSTR("inRadius"), DRONE_LINK_MSG_TYPE_FLOAT, inRadiusH);

  DEMCommandHandler goHomeH = std::bind(&NavModule::nav_goHome, this, _1, _2, _3, _4);
  _dem->registerCommand(ns, PSTR("goHome"), DEM_DATATYPE_NONE, goHomeH);
}

void NavModule::loop() {
  DroneModule::loop();

  // see if we have a valid location to set home
  if (_params[NAV_PARAM_HOME_E].data.f[0] == 0 &&
      _subs[NAV_SUB_LOCATION_E].param.data.f[0] != 0) {
    memcpy(_params[NAV_PARAM_HOME_E].data.c, _subs[NAV_SUB_LOCATION_E].param.data.c, 8);
    publishParamEntry(&_params[NAV_PARAM_HOME_E]);
  }

  // see if we have a valid location to set as last
  if (_params[NAV_PARAM_LAST_E].data.f[0] == 0 &&
      _subs[NAV_SUB_LOCATION_E].param.data.f[0] != 0) {
    memcpy(_params[NAV_PARAM_LAST_E].data.c, _subs[NAV_SUB_LOCATION_E].param.data.c, 8);
    publishParamEntry(&_params[NAV_PARAM_LAST_E]);
  }

  // update cross-track distance
  if (!_atTarget) {
    float d = getCrossTrackDistance();
    if (_subs[NAV_SUB_TARGET_E].param.data.f[2] > 0) d = d / _subs[NAV_SUB_TARGET_E].param.data.f[2];
    updateAndPublishParam(&_params[NAV_PARAM_CROSSTRACK_E], (uint8_t*)&d, sizeof(d));
  }
}


void NavModule::updateLast(boolean fromTarget) {
  if (fromTarget) {
    updateAndPublishParam(&_params[NAV_PARAM_LAST_E], (uint8_t *)&_subs[NAV_SUB_TARGET_E].param.data.uint8, 12);
  } else {
    // from current location
    if (_subs[NAV_SUB_LOCATION_E].param.data.f[0] != 0) {
      memcpy(_params[NAV_PARAM_LAST_E].data.c, _subs[NAV_SUB_LOCATION_E].param.data.c, 8);
      _params[NAV_PARAM_LAST_E].data.f[2] = 1; // 1m dummy origin radius
      publishParamEntry(&_params[NAV_PARAM_LAST_E]);
    }
  }
}

void NavModule::update() {
  if (!_setupDone) return;

  if (_params[NAV_PARAM_MODE_E].data.uint8[0] == NAV_IDLE) {

    float tempF = 0;

    //updateAndPublishParam(&_params[NAV_PARAM_HEADING_E], (uint8_t*)&tempF, sizeof(tempF));
    //updateAndPublishParam(&_params[NAV_PARAM_ADJ_HEADING_E], (uint8_t*)&tempF, sizeof(tempF));

    // make sure distance is set to zero
    updateAndPublishParam(&_params[NAV_PARAM_DISTANCE_E], (uint8_t*)&tempF, sizeof(tempF));

    return;
  }

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

  if (lat1 == 0 || lon1 == 0) {
    Log.noticeln("[Nav.u] invalid location");
    return;
  }

  float lat2 = _subs[NAV_SUB_TARGET_E].param.data.f[1];
  float lon2 = _subs[NAV_SUB_TARGET_E].param.data.f[0];

  if (lat2 == 0 || lon2 == 0) {
    Log.noticeln("[Nav.u] invalid target");
    return;
  }


  float R = RADIUS_OF_EARTH; // metres
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

  //_params[NAV_PARAM_HEADING_E].data.f[0] = h;
  // modify heading based on cross-track distance
  float crossTrackAdj = _params[NAV_PARAM_CROSSTRACK_E].data.f[0] * _params[NAV_PARAM_CORRECTION_E].data.f[0];
  if (crossTrackAdj > 90) crossTrackAdj = 90;
  if (crossTrackAdj < -90) crossTrackAdj = -90;
  float adjH = h + crossTrackAdj;

  // now calculate crosswind adjustment
  float w = _subs[NAV_SUB_WIND_E].param.data.f[0];
  float cw = _params[NAV_PARAM_CROSSWIND_E].data.f[0];

  // TODO - get wind speed and hull speed
  float windSpeed = 1;
  //float hullSpeed = 1;

  // TODO - account for hullSpeed in direction of current heading

  // -- calc adj Target for crosswind --
  // convert wind to vector, modify wind by crosswind factor
  float wr = degreesToRadians(w);
  float wv[2];
  wv[0] = cw * windSpeed * cos(wr);
  wv[1] = cw * windSpeed * sin(wr);

  // calc current heading vector
  float tr = degreesToRadians(adjH);
  float tv[2];
  tv[0] =  1 * cos(tr);
  tv[1] = 1 * sin(tr);

  // calc adj vector by summing
  float av[2];
  av[0] = wv[0] + tv[0];
  av[1] = wv[1] + tv[1];
  // calc adjusted heading
  adjH = radiansToDegrees(atan2(av[1], av[0]));


  updateAndPublishParam(&_params[NAV_PARAM_HEADING_E], (uint8_t*)&h, sizeof(h));
  updateAndPublishParam(&_params[NAV_PARAM_ADJ_HEADING_E], (uint8_t*)&adjH, sizeof(adjH));

  //_params[NAV_PARAM_DISTANCE_E].data.f[0] = d;
  updateAndPublishParam(&_params[NAV_PARAM_DISTANCE_E], (uint8_t*)&d, sizeof(d));

  // check to see if we've reached the waypoint
  // by comparing d (distance to go) to the target radius [2]
  if (d < _subs[NAV_SUB_TARGET_E].param.data.f[2]) {
    // TODO - we made it... now what?
    // just keep going?
    // or switch to some sort of loiter?
    _atTarget = true;
    updateLast( true );
  } else {
    _atTarget = false;
  }
}


float NavModule::getDistanceTo(float lon2, float lat2) {
  float lat1 = _subs[NAV_SUB_LOCATION_E].param.data.f[1];
  float lon1 = _subs[NAV_SUB_LOCATION_E].param.data.f[0];

  float R = RADIUS_OF_EARTH; // metres
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

  return d;
}


float NavModule::getCrossTrackDistance() {
  // calculate cross-track distance from current location to line between last waypoint and target waypoint
  // in meters

  if (_subs[NAV_SUB_TARGET_E].param.data.f[0] == 0 ||
      _params[NAV_PARAM_LAST_E].data.f[0] == 0 ||
    _subs[NAV_SUB_LOCATION_E].param.data.f[0] == 0) return 0;

  // local shortcuts
  double lon1 = _params[NAV_PARAM_LAST_E].data.f[0];
  double lat1 = _params[NAV_PARAM_LAST_E].data.f[1];
  double lon2 = _subs[NAV_SUB_TARGET_E].param.data.f[0];
  double lat2 = _subs[NAV_SUB_TARGET_E].param.data.f[1];
  double lon3 = _subs[NAV_SUB_LOCATION_E].param.data.f[0];
  double lat3 = _subs[NAV_SUB_LOCATION_E].param.data.f[1];

  double y = sin(lon3 - lon1) * cos(lat3);
  double x = cos(lat1) * sin(lat3) - sin(lat1) * cos(lat3) * cos(lat3 - lat1);
  double bearing13 = radiansToDegrees(atan2(y, x));
  bearing13 = fmod((bearing13 + 360), 360);

  double y2 = sin(lon2 - lon1) * cos(lat2);
  double x2 = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lat2 - lat1);
  double bearing12 = radiansToDegrees(atan2(y2, x2));
  bearing12 = fmod((bearing12 + 360), 360);

  // get distance from last to current location
  double distanceACbyE = getDistanceTo(lon1, lat1) / RADIUS_OF_EARTH;

  double d = -(asin(sin(distanceACbyE)*sin(degreesToRadians(bearing13)-degreesToRadians(bearing12))) * RADIUS_OF_EARTH);

  return d;
}


boolean NavModule::_goto(DRONE_LINK_PAYLOAD *payload, boolean continuation) {
  if (continuation) {
    float d =  getDistanceTo(_subs[NAV_SUB_TARGET_E].param.data.f[0], _subs[NAV_SUB_TARGET_E].param.data.f[1]);
    if (d <= _subs[NAV_SUB_TARGET_E].param.data.f[2]) {
      // made it, this command is done
      _atTarget = true;
      return true;
    } else {
      // still on our way
      _atTarget = false;
      return false;
    }

  } else {
    Log.noticeln("[Nav.goto]");
    memcpy(&_subs[NAV_SUB_TARGET_E].param.data, payload, sizeof(DRONE_LINK_PAYLOAD));
    // ensure mode is goto
    _params[NAV_PARAM_MODE_E].data.uint8[0] = NAV_GOTO;
    publishParamEntries();

    // do we need to update last for first time?
   if (_params[NAV_PARAM_LAST_E].data.f[0] == 0) {
     updateLast(false);
   }

    return false;
  }
}



boolean NavModule::nav_inRadius(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  Log.noticeln("[Nav.inRadius]");

  float d =  getDistanceTo(instr->msg.payload.f[0], instr->msg.payload.f[1]);

  _dem->dataStackPush( d <= instr->msg.payload.f[2] ? 1 : 0 , instr);

  if (d <= instr->msg.payload.f[2]) {
    updateLast(true);
  }

  return true;
}


boolean NavModule::nav_goto(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  return _goto(&instr->msg.payload, continuation);
}


boolean NavModule::nav_goHome(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  return _goto(&_params[NAV_PARAM_HOME_E].data, continuation);
}
