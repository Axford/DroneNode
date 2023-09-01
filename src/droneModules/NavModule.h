/*

@type          Nav
@inherits      Drone
@description   Managen navigation between waypoints accounting for crosswind drift

@guide >>>
Target waypoints are defined in terms of Lon Lat and target Radius in meters.
<<<

@config >>>
[Nav= 9]
  name= "Nav"
  interval= 50
  crosswind = 0.2
  correction = 20
  $location= @>7.8
  target= -1.82, 51.56, 100
  publish = target, location, heading, distance, mode, last, home, crosstrack

<<<

*/

#ifndef NAV_MODULE_H
#define NAV_MODULE_H

#include "../DroneModule.h"

// subs
// @sub 10;11;f;3;location;Current location from GPS.location
#define NAV_SUB_LOCATION           10  // current location from GPS
#define NAV_SUB_LOCATION_ADDR      11
#define NAV_SUB_LOCATION_E         0

// @sub 12;13;f;3;target;Target location, typically from Waypoint.target
#define NAV_SUB_TARGET             12  // target location
#define NAV_SUB_TARGET_ADDR        13
#define NAV_SUB_TARGET_E           1

// @sub 21;22;f;1;wind;Current wind direction to allow for crosswind compensation
#define NAV_SUB_WIND               21
#define NAV_SUB_WIND_ADDR          22
#define NAV_SUB_WIND_E             2

// @sub 25;26;f;1;satellites;Number of satellites for GPS lock (e.g. @>GPS.satellites)
#define NAV_SUB_SATELLITES         25
#define NAV_SUB_SATELLITES_ADDR    26
#define NAV_SUB_SATELLITES_E       3

#define NAV_SUBS                   4


// pubs
// @pub 8;f;1;heading;Target heading to reach target, feed to Sailor.target or TurnRate.target
#define NAV_PARAM_HEADING        8  // required heading
#define NAV_PARAM_HEADING_E      0

// @pub 9;f;1;distance;Distance to target in meters
#define NAV_PARAM_DISTANCE       9  // distance to go
#define NAV_PARAM_DISTANCE_E     1

// @pub 14;u8;1;mode;0=idle, 1=goto, 2=follow, 3=absolute course, 4=relative course, 5=backaway, 6=orbit
#define NAV_PARAM_MODE           14  // mode
#define NAV_PARAM_MODE_E         2

// @pub 15;f;3;last;Location when we last received a new target, used to plan track
#define NAV_PARAM_LAST           15  // last waypoint/location
#define NAV_PARAM_LAST_E         3

// @pub 16;f;3;home;First valid location received since boot, used as home location
#define NAV_PARAM_HOME           16  // home waypoint/location
#define NAV_PARAM_HOME_E         4

// @pub 17;f;1;crosstrack;crosstrack ratio indicating distance from ideal track
#define NAV_PARAM_CROSSTRACK     17  // cross-track ratio
#define NAV_PARAM_CROSSTRACK_E   5

// @pub 18;f;1;correction;How much to adjust heading to stay on ideal track
#define NAV_PARAM_CORRECTION     18  // cross-track correction factor
#define NAV_PARAM_CORRECTION_E   6

// @pub 19;f;1;crosswind;How much crosswind effect to account for - larger values cause heading to turn into the wind
#define NAV_PARAM_CROSSWIND      19  // how much crosswind effect to account for
#define NAV_PARAM_CROSSWIND_E    7

// @pub 20;f;1;adjHeading;Heading adjusted for crosswind, feed to Sailor.target or TurnRate.target
#define NAV_PARAM_ADJ_HEADING    20  // heading adj for crosswind
#define NAV_PARAM_ADJ_HEADING_E  8

// @pub 23;f;1;pitch;Pitch to target - assuming elevation in both location and target
#define NAV_PARAM_PITCH          23 
#define NAV_PARAM_PITCH_E        9

// @pub 24;f;2;limits;Pitch limits (min, max)
#define NAV_PARAM_LIMITS         24
#define NAV_PARAM_LIMITS_E       10

#define NAV_PARAM_ENTRIES        11

static const char NAV_STR_NAV[] PROGMEM = "Nav";

enum NAV_MODE {
  NAV_IDLE,
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
  float _progress;  // ratio of current distance to target divided by target radius
  float _lastTarget[3];  // internal tracker of the last target

public:

  NavModule(uint8_t id, DroneSystem* ds);

  void onParamWrite(DRONE_PARAM_ENTRY *param);
  void onSubReceived(DRONE_PARAM_SUB *sub);

  void setup();
  void loop();

  void updateLast(boolean fromTarget);

  void update();

  float getDistanceTo(float lon2, float lat2);
  float getCrossTrackDistance();

  boolean _goto(DRONE_LINK_PAYLOAD *payload, boolean continuation);

  //boolean nav_inRadius(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
  //boolean nav_goto(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
  //boolean nav_goHome(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
};

#endif
