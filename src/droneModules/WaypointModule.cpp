#include "WaypointModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"
#include <LITTLEFS.h>

WaypointModule::WaypointModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(WAYPOINT_STR_WAYPOINT));

   _waypoints = IvanLinkedList::LinkedList<WAYPOINT_MODULE_WAYPOINT>();


   // mgmt
   //_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, sizeof(WAYPOINT_STR_WAYPOINT));
   //strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, WAYPOINT_STR_WAYPOINT, sizeof(WAYPOINT_STR_WAYPOINT));


   // subs
   initSubs(WAYPOINT_SUBS);

   _subs[WAYPOINT_SUB_LOCATION_E].addrParam = WAYPOINT_SUB_LOCATION_ADDR;
   _subs[WAYPOINT_SUB_LOCATION_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WAYPOINT_SUB_LOCATION);
   _subs[WAYPOINT_SUB_LOCATION_E].param.name = FPSTR(STRING_LOCATION);
   _subs[WAYPOINT_SUB_LOCATION_E].param.nameLen = sizeof(STRING_LOCATION);
   _subs[WAYPOINT_SUB_LOCATION_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);


   // pubs
   initParams(WAYPOINT_PARAM_ENTRIES);

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

   loadWaypoints();
}


DEM_NAMESPACE* WaypointModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(WAYPOINT_STR_WAYPOINT,0,true);
}

void WaypointModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_LOCATION, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$location"), DRONE_LINK_MSG_TYPE_FLOAT, pha);

  dem->registerCommand(ns, STRING_MODE, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}


void WaypointModule::loadWaypoints() {
  // empty current list of waypoints
  _waypoints.clear();

  uint8_t waypoints = 0;

  // read waypoint.csv  file
    if (LITTLEFS.exists(F("/waypoint.csv"))) {
      File file = LITTLEFS.open(F("/waypoint.csv"), FILE_READ);

      if (!file) {
        Serial.println("[w.lW] Error opening waypoint.csv");
      } else {
        /*
        1) read first line (header) and parse column indices for lon, lat and radius
        2) read following lines and parse desired elements to populate linked list
        */
        
        uint32_t line = 0;
        char buffer[32];  // parsing buffer
        uint8_t bufLen = 0;  // how many valid chars are in the buffer
        uint8_t col = 0;

        uint8_t lonCol = 255;
        uint8_t latCol = 255;
        uint8_t radiusCol = 255;
        WAYPOINT_MODULE_WAYPOINT t;
        t.lat = 0;
        t.lon = 0;
        t.radius = 0;
        float f;
        uint8_t colsFound = 0;

        while (file.available()) {
          char c = file.read();

          // skip whitespace
          if (c == '\t' || c==' ' || c=='\r') {
            continue;
          }

          if (c == ',' || c == '\n' || !file.available()) {
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
              f = (float)atof(buffer);

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
              if (( c == '\n' || !file.available() ) && colsFound == 3) {
                // store waypoint
                _waypoints.add(t);
              }
            }

            if (c == '\n') {
              line++;
              col = 0;
              colsFound = 0;
            } else {
              col++;
            }
            bufLen = 0;  // reset ready for next col

          } else {
            // append to buffer
            if (bufLen < 31) {
              buffer[bufLen] = c;
              bufLen++;
              buffer[bufLen] = 0; // null terminate as we go
            }
          }
          

        }

        waypoints = _waypoints.size();

      }

      file.close();
    } else {
      Serial.println("[W.lW] No waypoint.csv");
    }


  // update number of waypoints
  updateAndPublishParam(&_params[WAYPOINT_PARAM_WAYPOINTS_E], (uint8_t*)&waypoints, sizeof(waypoints));
}


void WaypointModule::loop() {
  if (!_setupDone) return;

  uint8_t m = _params[WAYPOINT_PARAM_MODE_E].data.uint8[0];
  uint8_t waypoint = _params[WAYPOINT_PARAM_WAYPOINT_E].data.uint8[0];
  uint8_t waypoints = _params[WAYPOINT_PARAM_WAYPOINTS_E].data.uint8[0];

  if (m == WAYPOINT_MODE_RELOAD) {
    // reload waypoint.csv file
    loadWaypoints();

    m = WAYPOINT_MODE_NORMAL;

    // load first waypoint from file
    waypoint = 0;
    waypoints = _waypoints.size();

  } else {

    // see if we have reached current waypoint
    float d = calculateDistanceBetweenCoordinates(
      _subs[WAYPOINT_SUB_LOCATION_E].param.data.f[0],
      _subs[WAYPOINT_SUB_LOCATION_E].param.data.f[1],
      _params[WAYPOINT_PARAM_TARGET_E].data.f[0],
      _params[WAYPOINT_PARAM_TARGET_E].data.f[1]
    );

    if (d < _params[WAYPOINT_PARAM_TARGET_E].data.f[2]) {
      // select next waypoint
      if (waypoints > 0) {
        if (waypoint == waypoints-1) {
          // we've reached the last waypoint
          // should we loop?
          if (_params[WAYPOINT_PARAM_LOOP_E].data.uint8[0] == 1) {
            waypoint = 0;
          }
        } else waypoint++;
      }
    }

  }

  if (waypoints == 0) {
    waypoint = 0;
  } else {
    if (waypoint >= waypoints) waypoint = waypoints-1;
  }

  if (waypoints > 0 && waypoint < waypoints) {
    // update new target
    WAYPOINT_MODULE_WAYPOINT t = _waypoints.get(waypoint);
    updateAndPublishParam(&_params[WAYPOINT_PARAM_TARGET_E], (uint8_t*)&t, sizeof(t));
  }

  // update params
  updateAndPublishParam(&_params[WAYPOINT_PARAM_MODE_E], (uint8_t*)&m, sizeof(m));
  updateAndPublishParam(&_params[WAYPOINT_PARAM_WAYPOINT_E], (uint8_t*)&waypoint, sizeof(waypoint));
  updateAndPublishParam(&_params[WAYPOINT_PARAM_WAYPOINTS_E], (uint8_t*)&waypoints, sizeof(waypoints));
}
