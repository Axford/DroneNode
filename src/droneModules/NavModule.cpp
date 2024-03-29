#include "NavModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../strings.h"
#include "../navMath.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

// @type Nav

NavModule::NavModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(NAV_STR_NAV));

   // @default interval = 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   _atTarget = false;

   _progress = 10; // outside target radius by default
   _lastTarget[0] = 0;
   _lastTarget[1] = 0;
   _lastTarget[2] = 0;

   // subs
   initSubs(NAV_SUBS);

   _subs[NAV_SUB_LOCATION_E].addrParam = NAV_SUB_LOCATION_ADDR;
   _subs[NAV_SUB_LOCATION_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NAV_SUB_LOCATION);
   _subs[NAV_SUB_LOCATION_E].param.name = FPSTR(STRING_LOCATION);
   _subs[NAV_SUB_LOCATION_E].param.nameLen = sizeof(STRING_LOCATION);
   _subs[NAV_SUB_LOCATION_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);

   _subs[NAV_SUB_TARGET_E].addrParam = NAV_SUB_TARGET_ADDR;
   _subs[NAV_SUB_TARGET_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, NAV_SUB_TARGET);
   _subs[NAV_SUB_TARGET_E].param.name = FPSTR(STRING_TARGET);
   _subs[NAV_SUB_TARGET_E].param.nameLen = sizeof(STRING_TARGET);
   _subs[NAV_SUB_TARGET_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 16);
   // target: lon lat radius duration

   _subs[NAV_SUB_WIND_E].addrParam = NAV_SUB_WIND_ADDR;
   _subs[NAV_SUB_WIND_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NAV_SUB_WIND);
   _subs[NAV_SUB_WIND_E].param.name = FPSTR(STRING_WIND);
   _subs[NAV_SUB_WIND_E].param.nameLen = sizeof(STRING_WIND);
   _subs[NAV_SUB_WIND_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _subs[NAV_SUB_SATELLITES_E].addrParam = NAV_SUB_SATELLITES_ADDR;
   _subs[NAV_SUB_SATELLITES_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NAV_SUB_SATELLITES);
   _subs[NAV_SUB_SATELLITES_E].param.name = FPSTR(STRING_SATELLITES);
   _subs[NAV_SUB_SATELLITES_E].param.nameLen = sizeof(STRING_SATELLITES);
   _subs[NAV_SUB_SATELLITES_E].param.paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   // pubs
   initParams(NAV_PARAM_ENTRIES);

   _params[NAV_PARAM_HEADING_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, NAV_PARAM_HEADING);
   _params[NAV_PARAM_HEADING_E].name = FPSTR(STRING_HEADING);
   _params[NAV_PARAM_HEADING_E].nameLen = sizeof(STRING_HEADING);
   _params[NAV_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[NAV_PARAM_DISTANCE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, NAV_PARAM_DISTANCE);
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
   // @default correction=20
   _params[NAV_PARAM_CORRECTION_E].data.f[0] = 20;

   DRONE_PARAM_ENTRY *param;
   param = &_params[NAV_PARAM_CROSSWIND_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NAV_PARAM_CROSSWIND);
   setParamName(FPSTR(STRING_CROSSWIND), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   // @default crosswind=0.5
   _params[NAV_PARAM_CROSSWIND_E].data.f[0] = 0.5;

   param = &_params[NAV_PARAM_ADJ_HEADING_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, NAV_PARAM_ADJ_HEADING);
   setParamName(FPSTR(STRING_ADJ_HEADING), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[NAV_PARAM_PITCH_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, NAV_PARAM_PITCH);
   setParamName(FPSTR(STRING_PITCH), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[NAV_PARAM_LIMITS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NAV_PARAM_LIMITS);
   setParamName(FPSTR(STRING_LIMITS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   // @default limits=-25,10
   param->data.f[0] = -25;
   param->data.f[1] = 10;
}


void NavModule::onParamWrite(DRONE_PARAM_ENTRY *param) {
  DroneModule::onParamWrite(param);

  if (getDroneLinkMsgParam(param->paramPriority) == NAV_SUB_TARGET) {
    // update last location if target changes
    updateLast(false);

    // update _lastTarget
    _lastTarget[0] = _subs[NAV_SUB_TARGET_E].param.data.f[0];
    _lastTarget[1] = _subs[NAV_SUB_TARGET_E].param.data.f[1];
    _lastTarget[2] = _subs[NAV_SUB_TARGET_E].param.data.f[2];
  }
}


void NavModule::onSubReceived(DRONE_PARAM_SUB *sub) {
  DroneModule::onSubReceived(sub);

  if (sub->addrParam == NAV_SUB_TARGET_ADDR) {
    // check how close we were to the previous target... if virtually there, assume we had made and it a separate WayPoint module has updated the target
    // within 10% of target
    if (_progress < 1.1) {
      // target changed, so update last using _lastTarget
      _params[NAV_PARAM_LAST_E].data.f[0] = _lastTarget[0];
      _params[NAV_PARAM_LAST_E].data.f[1] = _lastTarget[1];
      _params[NAV_PARAM_LAST_E].data.f[2] = _lastTarget[2];
      publishParamEntry(&_params[NAV_PARAM_LAST_E]);
    } else {
      // target changed, so update last
      updateLast(false);
    }
    
    // update _lastTarget
    _lastTarget[0] = _subs[NAV_SUB_TARGET_E].param.data.f[0];
    _lastTarget[1] = _subs[NAV_SUB_TARGET_E].param.data.f[1];
    _lastTarget[2] = _subs[NAV_SUB_TARGET_E].param.data.f[2];
  }
}


void NavModule::setup() {
  DroneModule::setup();
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
  DroneModule::update();
  if (!_setupDone) return;

  // check for sufficient satellites, assuming we have a valid sub
  if (_subs[NAV_SUB_SATELLITES_E].received) {
    if (_subs[NAV_SUB_SATELLITES_E].param.data.f[0] < 9) {
      if (_mgmtParams[DRONE_MODULE_PARAM_STATUS_E].data.uint8[0] != DRONE_MODULE_STATUS_WAITING) {
        uint8_t temp = DRONE_MODULE_STATUS_WAITING;
        updateAndPublishParam(&_mgmtParams[DRONE_MODULE_PARAM_STATUS_E], (uint8_t*)&temp, sizeof(temp));

        float tempF = 0;
        // make sure distance is set to zero
        updateAndPublishParam(&_params[NAV_PARAM_DISTANCE_E], (uint8_t*)&tempF, sizeof(tempF));

        waiting();
      }
    } else {
      if (_mgmtParams[DRONE_MODULE_PARAM_STATUS_E].data.uint8[0] == DRONE_MODULE_STATUS_WAITING) {
        uint8_t temp = DRONE_MODULE_STATUS_ENABLED;
        updateAndPublishParam(&_mgmtParams[DRONE_MODULE_PARAM_STATUS_E], (uint8_t*)&temp, sizeof(temp));
        enable();
      }
    }
  }

  // abandon if Waiting
  if (_mgmtParams[DRONE_MODULE_PARAM_STATUS_E].data.uint8[0] == DRONE_MODULE_STATUS_WAITING) return;


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
  if (crossTrackAdj > 45) crossTrackAdj = 45;
  if (crossTrackAdj < -45) crossTrackAdj = -45;
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

  // calc pitch to target - negative pitches are down (target below location), positive are up
  if (d != 0) {
    // negative distances mean target is below location
    float elevationDifference = _subs[NAV_SUB_TARGET_E].param.data.f[2] - _subs[NAV_SUB_LOCATION_E].param.data.f[2];
    
    float pitch = atan2(elevationDifference, d) * 180 / PI;

    // constrain pitch
    pitch = constrain(pitch, _params[NAV_PARAM_LIMITS_E].data.f[0], _params[NAV_PARAM_LIMITS_E].data.f[1]);

    updateAndPublishParam(&_params[NAV_PARAM_PITCH_E], (uint8_t*)&pitch, sizeof(pitch));
  }

  // check to see if we've reached the waypoint
  // by comparing d (distance to go) to the target radius [2]
  if (_subs[NAV_SUB_TARGET_E].param.data.f[2] > 0) {
    _progress = d / _subs[NAV_SUB_TARGET_E].param.data.f[2];
  } else {
    _progress = 0;
  }

  if (_progress <= 1) {
    if (!_atTarget) {
      // We've made it...  update last and wait for a new target to be set
      _atTarget = true;
      updateLast( true );
    }
    
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
  // calculate approximate cross-track distance from current location to line between last waypoint and target waypoint
  // in meters

  /*
  this is a hacky alternative to the "proper" formula, but is more robust to calculation errors
  with limited floating point precision.  It produces a generous estimate of crosstrack
  distance (approximation causes crosstrack to be over-stated) that improves in accuracy as the current location approaches the target.  This generally causes tacks that are a little narrower than the target corridor when close to the starting point, but expand to fill the tacking corridor
  closer to the target.
  */

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

  GeographicPoint lp;
  lp.lon = _params[NAV_PARAM_LAST_E].data.f[0];
  lp.lat = _params[NAV_PARAM_LAST_E].data.f[1];


  // calc distance d1 from last to current
  double d1 = getDistanceTo(lon1, lat1);
  
  // calc initial bearing b1 from last to target
  double b1 = calculateInitialBearingBetweenCoordinates(lon1, lat1, lon2, lat2);

  // calc position p1 using b1 and d1
  GeographicPoint p1 = calculateDestinationFromDistanceAndBearing(lp, d1, b1);

  // calc bearing from b2 from last to current
  double b2 = calculateInitialBearingBetweenCoordinates(lon1, lat1, lon3, lat3);

  // get sign of crosstrack from shortest path between bearings
  float delta = shortestSignedDistanceBetweenCircularValues(b1, b2);


  // calc distance between current and p1
  return getDistanceTo(p1.lon, p1.lat) * (delta > 0 ? -1 : 1);
}

/*
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
  double bearing13 = (atan2(y, x));
  //bearing13 = fmod((bearing13 + 360), 360);

  double y2 = sin(lon2 - lon1) * cos(lat2);
  double x2 = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lat2 - lat1);
  double bearing12 = (atan2(y2, x2));
  //bearing12 = fmod((bearing12 + 360), 360);

  // get distance from last to current location
  double distanceACbyE = getDistanceTo(lon1, lat1) / RADIUS_OF_EARTH;

  double d = -(asin(sin(distanceACbyE)*sin((bearing13)-(bearing12))) * RADIUS_OF_EARTH);

  return d;
}
*/


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


/*
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
*/