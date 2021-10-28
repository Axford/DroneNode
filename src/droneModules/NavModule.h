/*

Waypoint navigation:
   * Define a series of target locations (lat,lon, radius)
   * Sub to current location
   * Generate a target heading and est distance

*/
#ifndef NAV_MODULE_H
#define NAV_MODULE_H

#include "../DroneModule.h"

// subs
#define NAV_SUB_LOCATION           10  // current location from GPS
#define NAV_SUB_LOCATION_ADDR      11
#define NAV_SUB_LOCATION_E         0

#define NAV_SUB_TARGET             12  // target location
#define NAV_SUB_TARGET_ADDR        13
#define NAV_SUB_TARGET_E           1

#define NAV_SUBS                   2


// pubs
#define NAV_PARAM_HEADING        8  // required heading
#define NAV_PARAM_HEADING_E      0

#define NAV_PARAM_DISTANCE       9  // distance to go
#define NAV_PARAM_DISTANCE_E     1

#define NAV_PARAM_MODE           14  // mode
#define NAV_PARAM_MODE_E         2

#define NAV_PARAM_LAST           15  // last waypoint/location
#define NAV_PARAM_LAST_E         3

#define NAV_PARAM_HOME           16  // home waypoint/location
#define NAV_PARAM_HOME_E         4

#define NAV_PARAM_ENTRIES        5

static const char NAV_STR_NAV[] PROGMEM = "Nav";

enum NAV_MODE {
  NAV_GOTO,
  NAV_FOLLOW,
  NAV_ABSCOURSE,
  NAV_RELCOURSE,
  NAV_BACKAWAY,
  NAV_ORBIT
};

class NavModule:  public DroneModule {
protected:

public:

  NavModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);
  ~NavModule();

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void setup();
  void loop();

  void updateLast(boolean fromTarget);

  void update();

  float getDistanceTo(float lon2, float lat2);

  boolean _goto(DRONE_LINK_PAYLOAD *payload, boolean continuation);

  boolean nav_inRadius(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
  boolean nav_goto(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
  boolean nav_goHome(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
};

#endif
