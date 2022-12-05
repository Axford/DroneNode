/*
@type Waypoint
@description Load and manage a series of waypoints from a CSV file

@guide >>>

pub
mode 0 = normal, 1 = reload waypoints
current waypoint number u8
number of waypoints u8
target - current loaded waypoint

sub
location

xxx
<<<

@config >>>
Waypoint.new x
  name "Waypoint"
  
.done
<<<

*/

#ifndef WAYPOINT_MODULE_H
#define WAYPOINT_MODULE_H

#include "../DroneModule.h"
#include "LinkedList.h"

// subs
// @sub 12;13;f;3;location;Location from GPS 
#define WAYPOINT_SUB_LOCATION          12
#define WAYPOINT_SUB_LOCATION_ADDR     13
#define WAYPOINT_SUB_LOCATION_E        0

#define WAYPOINT_SUBS                1

// pubs
// @pub 8;u8;1;mode;Mode 0=normal, 1=Reload waypoint.csv
#define WAYPOINT_PARAM_MODE       8
#define WAYPOINT_PARAM_MODE_E     0

// @pub 9;u8;1;waypoints;How many waypoints are loaded
#define WAYPOINT_PARAM_WAYPOINTS       9
#define WAYPOINT_PARAM_WAYPOINTS_E     1

// @pub 10;u8;1;waypoint;Current waypoint number
#define WAYPOINT_PARAM_WAYPOINT         10
#define WAYPOINT_PARAM_WAYPOINT_E       2

// @pub 11;f;3;target;Target location of current waypoint
#define WAYPOINT_PARAM_TARGET         11
#define WAYPOINT_PARAM_TARGET_E       3

#define WAYPOINT_PARAM_ENTRIES         4


#define WAYPOINT_MODE_NORMAL           0
#define WAYPOINT_MODE_RELOAD           1


// -----------------------------------------------------------------------------
struct WAYPOINT_MODULE_WAYPOINT {
  float lon;
  float lat;
  float radius;
};


static const char WAYPOINT_STR_WAYPOINT[] PROGMEM = "Waypoint";

class WaypointModule:  public DroneModule {
protected:
    IvanLinkedList::LinkedList<WAYPOINT_MODULE_WAYPOINT> _waypoints;

public:

  WaypointModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void loadWaypoints();

  void loop();
};

#endif
