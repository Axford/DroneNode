/*

Waypoint navigation:
   * Define a series of target locations (lat,lon, radius)
   * Sub to current location
   * Generate a target heading and est distance

*/
#ifndef WAYPOINT_NAV_MODULE_H
#define WAYPOINT_NAV_MODULE_H

#include "../DroneModule.h"


#define WAYPOINT_NAV_PARAM_TARGET         8  // target location
#define WAYPOINT_NAV_PARAM_TARGET_E       0

#define WAYPOINT_NAV_PARAM_LOCATION       9  // current location
#define WAYPOINT_NAV_PARAM_LOCATION_E     1

#define WAYPOINT_NAV_PARAM_HEADING        10  // required heading
#define WAYPOINT_NAV_PARAM_HEADING_E      2

#define WAYPOINT_NAV_PARAM_DISTANCE       11  // distance to go
#define WAYPOINT_NAV_PARAM_DISTANCE_E     3

#define WAYPOINT_NAV_PARAM_WAYPOINTS      12  // how many waypoints
#define WAYPOINT_NAV_PARAM_WAYPOINTS_E    4

#define WAYPOINT_NAV_PARAM_WAYPOINT       13  // current index
#define WAYPOINT_NAV_PARAM_WAYPOINT_E     5

#define WAYPOINT_NAV_PARAM_LOOPTO         14  // current index
#define WAYPOINT_NAV_PARAM_LOOPTO_E       6

#define WAYPOINT_NAV_PARAM_ENTRIES        7

#define WAYPOINT_NAV_PARAM_WAYPOINTS_START   100  // param address of first waypoint


static const char WAYPOINT_NAV_STR_WAYPOINT_NAV[] PROGMEM = "WaypointNav";

class WaypointNavModule:  public DroneModule {
protected:
  DRONE_LINK_ADDR _location;  // current location

  boolean _updateNeeded;
  uint8_t _allocatedWaypoints;  // how much memory have we allocated
  DRONE_PARAM_ENTRY *_waypoints;

  uint8_t _publishWaypoint;  // index to which waypoint to publish next

public:

  WaypointNavModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);

  void initWaypoint(uint8_t i);

  void onParamWrite(DRONE_PARAM_ENTRY *param);

  void loadConfiguration(JsonObject &obj);

  boolean publishParamEntries();

  void handleLinkMessage(DroneLinkMsg *msg);

  void setup();
  void loop();

  void update();

  void selectWaypoint(uint8_t i);
};

#endif
