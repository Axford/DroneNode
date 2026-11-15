#include "../DroneConsole.h"
#include "WaypointModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"
#include "Preferences.h"
#include <mbedtls/sha256.h>

WaypointModule::WaypointModule(uint8_t id, DroneSystem* ds, Registration registration):
  DroneModule(id, ds, registration)
 {
   if (!_mgmtParams) return;
   setTypeName(FPSTR(WAYPOINT_STR_WAYPOINT));

    // @default interval = 1000
    _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;  // 1 sec

   _waypoints = IvanLinkedList::LinkedList<WAYPOINT_MODULE_WAYPOINT>();
   _lastStoredWaypoint = 0;

   _totalDistance = 0;
   _distanceRemaining = 0;
   _distanceToNext = 0;

   _firstDistanceRemaining = 0;
   _firstDistanceRemainingTime = 0;

   // subs
   initSubs(WAYPOINT_SUBS);
   if (!_subs) return;

   _subs[WAYPOINT_SUB_LOCATION_E].addrParam = WAYPOINT_SUB_LOCATION_ADDR;
   _subs[WAYPOINT_SUB_LOCATION_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WAYPOINT_SUB_LOCATION);
   _subs[WAYPOINT_SUB_LOCATION_E].param.name = FPSTR(STRING_LOCATION);
   _subs[WAYPOINT_SUB_LOCATION_E].param.nameLen = sizeof(STRING_LOCATION);
   _subs[WAYPOINT_SUB_LOCATION_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);


   // pubs
   initParams(WAYPOINT_PARAM_ENTRIES);
   if (!_params) return;

   _params[WAYPOINT_PARAM_MODE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WAYPOINT_PARAM_MODE);
   _params[WAYPOINT_PARAM_MODE_E].name = FPSTR(STRING_MODE);
   _params[WAYPOINT_PARAM_MODE_E].nameLen = sizeof(STRING_MODE);
   _params[WAYPOINT_PARAM_MODE_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[WAYPOINT_PARAM_MODE_E].data.uint8[0] = WAYPOINT_MODE_RELOAD;

   _params[WAYPOINT_PARAM_WAYPOINTS_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WAYPOINT_PARAM_WAYPOINTS);
   _params[WAYPOINT_PARAM_WAYPOINTS_E].name = FPSTR(STRING_WAYPOINTS);
   _params[WAYPOINT_PARAM_WAYPOINTS_E].nameLen = sizeof(STRING_WAYPOINTS);
   _params[WAYPOINT_PARAM_WAYPOINTS_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[WAYPOINT_PARAM_WAYPOINTS_E].data.uint8[0] = 0;

   _params[WAYPOINT_PARAM_WAYPOINT_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, WAYPOINT_PARAM_WAYPOINT);
   _params[WAYPOINT_PARAM_WAYPOINT_E].name = FPSTR(STRING_WAYPOINT);
   _params[WAYPOINT_PARAM_WAYPOINT_E].nameLen = sizeof(STRING_WAYPOINT);
   _params[WAYPOINT_PARAM_WAYPOINT_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[WAYPOINT_PARAM_WAYPOINT_E].data.uint8[0] = 0;

   _params[WAYPOINT_PARAM_TARGET_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, WAYPOINT_PARAM_TARGET);
   _params[WAYPOINT_PARAM_TARGET_E].name = FPSTR(STRING_TARGET);
   _params[WAYPOINT_PARAM_TARGET_E].nameLen = sizeof(STRING_TARGET);
   _params[WAYPOINT_PARAM_TARGET_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[WAYPOINT_PARAM_TARGET_E].data.f[0] = 0;
   _params[WAYPOINT_PARAM_TARGET_E].data.f[1] = 0;
   _params[WAYPOINT_PARAM_TARGET_E].data.f[2] = 0;

   _params[WAYPOINT_PARAM_LOOP_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WAYPOINT_PARAM_LOOP);
   _params[WAYPOINT_PARAM_LOOP_E].name = FPSTR(STRING_LOOP);
   _params[WAYPOINT_PARAM_LOOP_E].nameLen = sizeof(STRING_LOOP);
   _params[WAYPOINT_PARAM_LOOP_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[WAYPOINT_PARAM_LOOP_E].data.uint8[0] = 0;

   _params[WAYPOINT_PARAM_DISTANCE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WAYPOINT_PARAM_DISTANCE);
   _params[WAYPOINT_PARAM_DISTANCE_E].name = FPSTR(STRING_DISTANCE);
   _params[WAYPOINT_PARAM_DISTANCE_E].nameLen = sizeof(STRING_DISTANCE);
   _params[WAYPOINT_PARAM_DISTANCE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[WAYPOINT_PARAM_DISTANCE_E].data.f[0] = 0;
   _params[WAYPOINT_PARAM_DISTANCE_E].data.f[1] = 0;
   _params[WAYPOINT_PARAM_DISTANCE_E].data.f[2] = 0;

   _params[WAYPOINT_PARAM_SPEED_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WAYPOINT_PARAM_SPEED);
   _params[WAYPOINT_PARAM_SPEED_E].name = FPSTR(STRING_SPEED);
   _params[WAYPOINT_PARAM_SPEED_E].nameLen = sizeof(STRING_SPEED);
   _params[WAYPOINT_PARAM_SPEED_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   auto &digest = _params[WAYPOINT_PARAM_ROUTE_DIGEST_E];
   digest.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WAYPOINT_PARAM_ROUTE_DIGEST);
   setParamName(F("routeDigest"), &digest);
   digest.paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);
   memset(digest.data.c, 0, 16);

   auto &loops = _params[WAYPOINT_PARAM_LOOPS_E];
   loops.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, WAYPOINT_PARAM_LOOPS);
   setParamName(F("loops"), &loops);
   loops.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   loops.data.uint32[0] = 0;
}



bool WaypointModule::loadWaypoints(bool strict) {
  // empty current list of waypoints
  _waypoints.clear();
  _finalArrivalCounted = false;
  uint32_t loops = 0;
  updateAndPublishParam(&_params[WAYPOINT_PARAM_LOOPS_E], reinterpret_cast<uint8_t *>(&loops), sizeof(loops));

  uint8_t waypoints = 0;
  uint8_t digest[32] = {};

  WAYPOINT_MODULE_WAYPOINT lastW;
  lastW.lat = 0;
  lastW.lon = 0;
  lastW.cumulativeDistance = 0;
  lastW.distanceFromLast = 0;
  lastW.distanceRemaining = 0;

  // read waypoint.csv  file
  if (LITTLEFS.exists(F("/waypoint.csv"))) {
    File file = LITTLEFS.open(F("/waypoint.csv"), FILE_READ);

    if (!file) {
      DroneConsole.println("[w.lW] Error opening waypoint.csv");
    } else {
      /*
      1) read first line (header) and parse column indices for lon, lat and radius
      2) read following lines and parse desired elements to populate linked list
      */
      
      uint32_t line = 0;
      char buffer[32] = {};  // parsing buffer
      uint8_t bufLen = 0;  // how many valid chars are in the buffer
      uint8_t col = 0;

      uint8_t lonCol = 255;
      uint8_t latCol = 255;
      uint8_t radiusCol = 255;
      WAYPOINT_MODULE_WAYPOINT t;
      t.lat = 0;
      t.lon = 0;
      t.radius = 0;
      t.cumulativeDistance = lastW.cumulativeDistance;
      t.distanceFromLast = 0;
      t.distanceRemaining = 0;
      float f;
      uint8_t colsFound = 0;

      mbedtls_sha256_context sha;
      mbedtls_sha256_init(&sha);
      bool complete = mbedtls_sha256_starts_ret(&sha, 0) == 0;
      const auto size = file.size();
      if (strict && size > 16384) complete = false;
      size_t read = 0;
      uint8_t bytes[256];
      while (read < size && (!strict || complete)) {
        const auto length = file.read(bytes, size - read < sizeof(bytes) ? size - read : sizeof(bytes));
        if (length == 0) break;
        complete = mbedtls_sha256_update_ret(&sha, bytes, length) == 0 && complete;
        read += length;
        for (size_t index = 0; index < length; ++index) {
          char c = bytes[index];
          const bool last = read == size && index + 1 == length;
          if (last && c != ',' && c != '\n' && c != '\r' && c != '\t' && c != ' ') {
            if (bufLen < 31) buffer[bufLen++] = c;
            c = '\n';
          }

          // skip whitespace
          if (c == '\t' || c==' ' || c=='\r') {
            if (!last) continue;
            c = '\n';
          }

          if (c == ',' || c == '\n' || last) {
            buffer[bufLen] = 0;
            if (line == 0) {
              // header
              if (strcmp(buffer, "lon") == 0) {
                lonCol = col;
              } else if (strcmp(buffer, "lat") == 0) {
                latCol = col;
              } else if (strcmp(buffer, "radius") == 0) {
                radiusCol = col;
              };

            } else {
              // data row
              // attempt to convert buffer into a float
              char* end;
              f = strtof(buffer, &end);
              if (strict && (col == lonCol || col == latCol || col == radiusCol) &&
                  (end == buffer || *end || !isfinite(f) ||
                   (col == lonCol && fabsf(f) > 180) || (col == latCol && fabsf(f) > 90) ||
                   (col == radiusCol && f <= 0))) complete = false;

              // see which column this relates to
              if (col == lonCol) {
                t.lon = f;
                colsFound++;
              } else if (col == latCol) {
                t.lat = f;
                colsFound++;
              } else if (col == radiusCol) {
                t.radius = f;
                colsFound++;
              };

              // have we found a complete row?
              if (( c == '\n' || last ) && colsFound == 3) {
                // calculate distances
                if (strict ? _waypoints.size() > 0 : lastW.lon != 0)
                  t.distanceFromLast = calculateDistanceBetweenCoordinates(lastW.lon, lastW.lat, t.lon, t.lat);
                t.cumulativeDistance += t.distanceFromLast;

                // store waypoint
                if ((strict && _waypoints.size() == UINT8_MAX) || !_waypoints.add(t)) complete = false;

                memcpy(&lastW, &t, sizeof(lastW));
              }
            }

            if (c == '\n') {
              if (strict && ((line == 0 && (lonCol == 255 || latCol == 255 || radiusCol == 255)) ||
                             (line != 0 && colsFound != 3))) complete = false;
              if (strict) t.distanceFromLast = 0;
              line++;
              col = 0;
              colsFound = 0;
            } else {
              if (strict && col == 254) complete = false;
              col++;
            }
            bufLen = 0;  // reset ready for next col

          } else {
            // append to buffer
            if (bufLen < 31) {
              buffer[bufLen] = c;
              bufLen++;
              buffer[bufLen] = 0; // null terminate as we go
            } else if (strict) complete = false;
          }


        }
      }
      if (!complete || read != size || (strict && _waypoints.size() > UINT8_MAX)) _waypoints.clear();
      if (_waypoints.size() > 0 && _waypoints.size() <= UINT8_MAX &&
          mbedtls_sha256_finish_ret(&sha, digest) != 0) memset(digest, 0, sizeof(digest));
      mbedtls_sha256_free(&sha);
      waypoints = _waypoints.size();

    }

    file.close();
  } else {
    DroneConsole.println("[W.lW] No waypoint.csv");
  }

  // loop back over waypoints and calculate distance remaining
  float distanceRemaining = 0;
  for (int i = _waypoints.size()-1; i>=0; i--) {
    WAYPOINT_MODULE_WAYPOINT t = _waypoints.get(i);

    t.distanceRemaining = distanceRemaining;
    distanceRemaining += t.distanceFromLast;
    
    _waypoints.set(i,t);
  }

  _totalDistance = lastW.cumulativeDistance;

  updateAndPublishParam(&_params[WAYPOINT_PARAM_ROUTE_DIGEST_E], digest, 16);

  // update number of waypoints
  updateAndPublishParam(&_params[WAYPOINT_PARAM_WAYPOINTS_E], (uint8_t*)&waypoints, sizeof(waypoints));
  return waypoints != 0;
}


void WaypointModule::setup() {
  DroneModule::setup();

  if (_liveLogic) return;

  loadWaypoints();

  // see if we have a stored waypoint number to jump to
  Preferences pref; 
  // use module name as preference namespace
  pref.begin(_mgmtParams[DRONE_MODULE_PARAM_NAME_E].data.c, true);
  uint16_t w = pref.getUShort("waypoint", 1000);
  if (w < _params[WAYPOINT_PARAM_WAYPOINTS_E].data.uint8[0]) {
    _params[WAYPOINT_PARAM_WAYPOINT_E].data.uint8[0] = w;
    _lastStoredWaypoint = w;
  }
  pref.end();
}

bool WaypointModule::validateLiveConfiguration() const {
  return _params[WAYPOINT_PARAM_MODE_E].data.uint8[0] == WAYPOINT_MODE_NORMAL &&
         _params[WAYPOINT_PARAM_WAYPOINT_E].data.uint8[0] == 0 &&
         _params[WAYPOINT_PARAM_LOOP_E].data.uint8[0] <= 1;
}

bool WaypointModule::canRetireLive() const {
  return _liveLogic || !_enabled || _activeWaypoint < 0;
}

bool WaypointModule::prepareLiveConfiguration() {
  return validateLiveConfiguration() && loadWaypoints(true);
}

bool WaypointModule::resumeLiveRoute() {
  if (!_liveLogic || !_setupDone || !_enabled || !_waypoints.size()) return false;
  _liveRouteResumed = true;
  return true;
}


void WaypointModule::onParamWrite(DRONE_PARAM_ENTRY *param) {
  DroneModule::onParamWrite(param);
  const auto id = getDroneLinkMsgParam(param->paramPriority);
  if (id == WAYPOINT_PARAM_MODE || id == WAYPOINT_PARAM_WAYPOINT) _activeWaypoint = -1;
}

void WaypointModule::handleLinkMessage(DroneLinkMsg *msg) {
  if (msg->node() == _dlm->node() && msg->channel() == _id && msg->param() == WAYPOINT_PARAM_LOOPS &&
      msg->type() == DRONE_LINK_MSG_TYPE_UINT32_T && msg->length() != sizeof(uint32_t)) return;
  DroneModule::handleLinkMessage(msg);
}

uint32_t WaypointModule::activeVisit(const float *target) {
  if (_liveLogic || !_setupDone || !_enabled || _activeWaypoint < 0 ||
      _params[WAYPOINT_PARAM_MODE_E].data.uint8[0] != WAYPOINT_MODE_NORMAL ||
      _params[WAYPOINT_PARAM_WAYPOINT_E].data.uint8[0] != _activeWaypoint ||
      memcmp(target, _params[WAYPOINT_PARAM_TARGET_E].data.f, 3 * sizeof(float)) != 0) return 0;
  return _visit;
}

bool WaypointModule::moveActiveWaypoint(uint32_t visit, float longitude, float latitude) {
  auto &target = _params[WAYPOINT_PARAM_TARGET_E];
  if (!visit || activeVisit(target.data.f) != visit) return false;
  float moved[] = {longitude, latitude, target.data.f[2]};
  updateAndPublishParam(&target, reinterpret_cast<uint8_t *>(moved), sizeof(moved));
  return true;
}

void WaypointModule::publishTarget(uint8_t waypoint) {
  const auto saved = _waypoints.get(waypoint);
  float target[] = {saved.lon, saved.lat, saved.radius};
  updateAndPublishParam(&_params[WAYPOINT_PARAM_TARGET_E], reinterpret_cast<uint8_t *>(target), sizeof(target));
}

void WaypointModule::loop() {
  DroneModule::loop();
  if (!_setupDone || (_liveLogic && !_liveRouteResumed)) return;

  uint8_t m = _params[WAYPOINT_PARAM_MODE_E].data.uint8[0];
  uint8_t waypoint = _params[WAYPOINT_PARAM_WAYPOINT_E].data.uint8[0];
  uint8_t waypoints = _params[WAYPOINT_PARAM_WAYPOINTS_E].data.uint8[0];

  bool beginVisit = _activeWaypoint != waypoint;
  if (waypoints == 0 || waypoint != waypoints - 1) _finalArrivalCounted = false;
  float newSpeed = _params[WAYPOINT_PARAM_SPEED_E].data.f[0];

  if (m == WAYPOINT_MODE_RELOAD) {
    // reload waypoint.csv file
    loadWaypoints();

    m = WAYPOINT_MODE_NORMAL;

    // load first waypoint from file
    waypoint = 0;
    waypoints = _waypoints.size();
    beginVisit = true;

  } else if (waypoints > 0 && !beginVisit) {

    // ensure we have valid locations
    if (_subs[WAYPOINT_SUB_LOCATION_E].param.data.f[0] != 0 && 
       _params[WAYPOINT_PARAM_TARGET_E].data.f[0] != 0) {

      // see if we have reached current waypoint
      float d = calculateDistanceBetweenCoordinates(
        _subs[WAYPOINT_SUB_LOCATION_E].param.data.f[0],
        _subs[WAYPOINT_SUB_LOCATION_E].param.data.f[1],
        _params[WAYPOINT_PARAM_TARGET_E].data.f[0],
        _params[WAYPOINT_PARAM_TARGET_E].data.f[1]
      );

      _distanceToNext = d;

      // calc distance remaining
      _distanceRemaining = d;
      WAYPOINT_MODULE_WAYPOINT t = _waypoints.get(waypoint);
      _distanceRemaining += t.distanceRemaining;

      // update distances
      float distances[3] = { _distanceToNext, _distanceRemaining, _totalDistance };
      updateAndPublishParam(&_params[WAYPOINT_PARAM_DISTANCE_E], (uint8_t*)&distances, sizeof(distances));

      const float radius = _params[WAYPOINT_PARAM_TARGET_E].data.f[2];
      if (waypoints == 1 && d >= radius) _finalArrivalCounted = false;
      if (d < radius) {
        // select next waypoint
        if (waypoints > 0) {
          if (waypoint == waypoints-1) {
            if (!_finalArrivalCounted) {
              _finalArrivalCounted = true;
              auto &parameter = _params[WAYPOINT_PARAM_LOOPS_E];
              uint32_t loops = parameter.data.uint32[0];
              if (loops < UINT32_MAX) ++loops;
              updateAndPublishParam(&parameter, reinterpret_cast<uint8_t *>(&loops), sizeof(loops));
            }
            // we've reached the last waypoint
            // should we loop?
            if (_params[WAYPOINT_PARAM_LOOP_E].data.uint8[0] == 1) {
              waypoint = 0;
              _firstDistanceRemaining = 0;
              beginVisit = true;
            }
          } else {
            waypoint++;
            beginVisit = true;
          }
        }
      }

      // update speed estimate
      if (_firstDistanceRemaining == 0 && _distanceRemaining > 0) {
        _firstDistanceRemaining = _distanceRemaining;
        _firstDistanceRemainingTime = millis();
      } else if (_distanceRemaining > 0) {
        //Log.noticeln("[WP] dist: %F, %F", _firstDistanceRemaining, _distanceRemaining);
        // update speed calc, must have got closer by at least 100m
        if (_distanceRemaining <  _firstDistanceRemaining - 100) {
          // calc time delta
          float dt = (millis() - _firstDistanceRemainingTime) / 1000;
          if (dt < 0) {
            // timer overflow... reset calculations
            _firstDistanceRemaining = 0;
          } else if (dt > 0) {
            newSpeed = (_firstDistanceRemaining - _distanceRemaining) / dt;
            //Log.noticeln("[WP] speed: %F, %F", newSpeed, dt);
          }
        }
      }

      updateAndPublishParam(&_params[WAYPOINT_PARAM_SPEED_E], (uint8_t*)&newSpeed, sizeof(newSpeed));
    }
  }

  if (waypoints == 0) {
    waypoint = 0;
  } else {
    if (waypoint >= waypoints) waypoint = waypoints-1;
  }

  if (waypoints > 0 && waypoint < waypoints) {
    if (beginVisit) {
      _activeWaypoint = waypoint;
      if (++_visit == 0) ++_visit;
      publishTarget(waypoint);
    }

    // do we need to store the waypoint in flash?
    if (!_liveLogic && waypoint != _lastStoredWaypoint) {
      _lastStoredWaypoint = waypoint;
      Preferences pref;
      pref.begin(_mgmtParams[DRONE_MODULE_PARAM_NAME_E].data.c, false);
      pref.putUShort("waypoint", waypoint);
      pref.end();
    }
  } else {
    _activeWaypoint = -1;
    float unset[3] = {};
    updateAndPublishParam(&_params[WAYPOINT_PARAM_TARGET_E], reinterpret_cast<uint8_t *>(unset), sizeof(unset));
  }

  // update params
  updateAndPublishParam(&_params[WAYPOINT_PARAM_MODE_E], (uint8_t*)&m, sizeof(m));
  updateAndPublishParam(&_params[WAYPOINT_PARAM_WAYPOINT_E], (uint8_t*)&waypoint, sizeof(waypoint));
  updateAndPublishParam(&_params[WAYPOINT_PARAM_WAYPOINTS_E], (uint8_t*)&waypoints, sizeof(waypoints));
}
