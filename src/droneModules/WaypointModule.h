/*
@type Waypoint
@inherits Drone
@description Load and manage a series of waypoints from a CSV file

@guide >>>

<p>Load and manage stepping through a series of waypoints, defined in /waypoint.csv.</p>

<p>Example:</p>
<pre>
lon,lat,radius
-1.744739150337,51.541523406305,10.0
-1.744404186599,51.542305699213,15.0
-1.746584868898,51.542284441432,10.0
-1.746516508951,51.541510651418,10.0
</pre>
<<<

@config >>>
[Waypoint=4]
  name=Waypoint
  $location=@>50.9
  publish =mode, waypoint, waypoints, target, location
<<<

*/

#ifndef WAYPOINT_MODULE_H
#define WAYPOINT_MODULE_H

#include "../DroneModule.h"
#include "LinkedList.h"

// subs
// @sub 12;13;f;3;location;Location from GPS 
#define WAYPOINT_SUB_LOCATION         12
#define WAYPOINT_SUB_LOCATION_ADDR    13
#define WAYPOINT_SUB_LOCATION_E       0

#define WAYPOINT_SUBS                 1

// pubs
// @pub 8;u8;1;mode;Mode 0=normal, 1=Reload waypoint.csv
#define WAYPOINT_PARAM_MODE           8
#define WAYPOINT_PARAM_MODE_E         0

// @pub 9;u8;1;waypoints;How many waypoints are loaded
#define WAYPOINT_PARAM_WAYPOINTS      9
#define WAYPOINT_PARAM_WAYPOINTS_E    1

// @pub 10;u8;1;waypoint;Current waypoint number
#define WAYPOINT_PARAM_WAYPOINT       10
#define WAYPOINT_PARAM_WAYPOINT_E     2

// @pub 11;f;3;target;Target location of current waypoint, feed to Nav.target
#define WAYPOINT_PARAM_TARGET         11
#define WAYPOINT_PARAM_TARGET_E       3

// @pub 14;u8;1;loop;0=stop at end of file, 1=loop
#define WAYPOINT_PARAM_LOOP           14
#define WAYPOINT_PARAM_LOOP_E         4

// @pub 15;f;3;distance;[0] distance to next waypoint, [1] distance remaining on total path [2] Cumulative length of waypoint path in meters
#define WAYPOINT_PARAM_DISTANCE       15
#define WAYPOINT_PARAM_DISTANCE_E     5

// @pub 16;f;1;speed;Approx speed along waypoint path in meters per second
#define WAYPOINT_PARAM_SPEED          16
#define WAYPOINT_PARAM_SPEED_E        6

#define WAYPOINT_PARAM_ENTRIES        7

#define WAYPOINT_MODE_NORMAL          0
#define WAYPOINT_MODE_RELOAD          1


// -----------------------------------------------------------------------------
struct WAYPOINT_MODULE_WAYPOINT {
  float lon;
  float lat;
  float radius;
  float distanceFromLast;
  float cumulativeDistance;
  float distanceRemaining;
};


static const char WAYPOINT_STR_WAYPOINT[] PROGMEM = "Waypoint";

class WaypointModule:  public DroneModule {
protected:
    uint8_t _lastStoredWaypoint;
    IvanLinkedList::LinkedList<WAYPOINT_MODULE_WAYPOINT> _waypoints;

    float _totalDistance;
    float _distanceRemaining;
    float _distanceToNext;

    // to determine speed
    float _firstDistanceRemaining;  // what was the first valid distance remaining we recorded
    uint32_t _firstDistanceRemainingTime;  // what millis() did we first record a valid distance remaining
   
public:

  WaypointModule(uint8_t id, DroneSystem* ds);

  void loadWaypoints();
  void setup();

  void loop();
};

#endif
