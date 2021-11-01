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

#define NAV_SUB_WIND               21
#define NAV_SUB_WIND_ADDR          22
#define NAV_SUB_WIND_E             2

#define NAV_SUBS                   3


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

#define NAV_PARAM_CROSSTRACK     17  // cross-track ratio
#define NAV_PARAM_CROSSTRACK_E   5

#define NAV_PARAM_CORRECTION     18  // cross-track correction factor
#define NAV_PARAM_CORRECTION_E   6

#define NAV_PARAM_CROSSWIND      19  // how much crosswind effect to account for
#define NAV_PARAM_CROSSWIND_E    7

#define NAV_PARAM_ADJ_HEADING     20  // heading adj for crosswind
#define NAV_PARAM_ADJ_HEADING_E   8

#define NAV_PARAM_ENTRIES        9

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
  boolean _atTarget;  // true if we've reached the current target

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
  float getCrossTrackDistance();

  boolean _goto(DRONE_LINK_PAYLOAD *payload, boolean continuation);

  boolean nav_inRadius(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
  boolean nav_goto(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
  boolean nav_goHome(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
};

#endif
