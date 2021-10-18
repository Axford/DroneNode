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

#define NAV_PARAM_MODE           10  // mode
#define NAV_PARAM_MODE_E         2

#define NAV_PARAM_ENTRIES        3

static const char NAV_STR_NAV[] PROGMEM = "Nav";

enum NAV_MODE {
  NAV_TO,
  NAV_FOLLOW,
  NAV_ABSCOURSE,
  NAV_RELCOURSE,
  NAV_BACKAWAY,
  NAV_ORBIT
};

class NavModule:  public DroneModule {
protected:
  boolean _updateNeeded;

public:

  NavModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);
  ~NavModule();

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void setup();
  void loop();

  void update();
};

#endif
