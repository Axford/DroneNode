/*
@type Sailor
@inherits Drone
@description Control a typical sailing boat

@guide >>>
Sailor controller

subs
* target heading (from nav module)
* wind direction (from wind sensor)
* cross-track factor (from nav module)
* ocean current vector - TODO

pubs
* heading (course to sail) - fed to rudder
* sheet extension (from 0 - close haul to 1 - run) - fed to sheet actuator

config (pubs)
* polar - sailing performance for different headings relative to wind
   - modelled as 16 segments in the clockwise 180 degree region (assumes mirrored performance)
   - each segment is 180/16 degree = 11.25 degree arc, centre of segment should be considered the target heading to achieve associated performance
   - performance stored as a single uint8_t value = 0..255

<<<

@config >>>
[Sailor=12]
  name="Sailor"
  interval=1000
  polar=0, 0, 0, 200,   255, 255, 255, 250,  240, 230, 220, 210,  200, 180, 160, 140
  crosswind=0.5
  $wind=@>50.10
  $target=@>Nav.heading
  $heading=@>50.8
  $crosstrack=@>Nav.crosstrack
  publish=target, heading, wind, crosstrack, course, sheet, polar
<<<

*/

#ifndef SAILOR_MODULE_H
#define SAILOR_MODULE_H

#include "../DroneModule.h"

// subs
// @sub 8;9;f;1;target;Target bearing, typically Nav.heading
#define SAILOR_SUB_TARGET              8
#define SAILOR_SUB_TARGET_ADDR         9
#define SAILOR_SUB_TARGET_E            0

// @sub 10;11;f;1;heading;Current heading, typically Compass.heading
#define SAILOR_SUB_HEADING             10
#define SAILOR_SUB_HEADING_ADDR        11
#define SAILOR_SUB_HEADING_E           1

// @sub 12;13;f;1;wind;Current wind direction, typically Wind.wind
#define SAILOR_SUB_WIND                12
#define SAILOR_SUB_WIND_ADDR           13
#define SAILOR_SUB_WIND_E              2

// @sub 14;15;f;1;crosstrack;Current crosstrack from Nav.crosstrack
#define SAILOR_SUB_CROSSTRACK          14
#define SAILOR_SUB_CROSSTRACK_ADDR     15
#define SAILOR_SUB_CROSSTRACK_E        3

#define SAILOR_SUBS                    4


//pubs
// @pub 16;f;1;course;Selected course/bearing - feed to Turnrate
#define SAILOR_PARAM_COURSE            16
#define SAILOR_PARAM_COURSE_E          0

// @pub 17;f;1;sheet;Sheet servo control value in range -1 (close haul) to 1 (run)
#define SAILOR_PARAM_SHEET             17
#define SAILOR_PARAM_SHEET_E           1

// @pub 18;u8;16;polar;Polar map for starboard wind (0-180 degrees), mirrored for port.  0=stall, 255=max relative speed
#define SAILOR_PARAM_POLAR             18
#define SAILOR_PARAM_POLAR_E           2

// @pub 19;u8;16;speed;polar plot of estimated speeds by heading - 0-180
#define SAILOR_PARAM_SPEED             19   
#define SAILOR_PARAM_SPEED_E           3

// @pub 20;u8;16;speed2;polar plot of estimated speeds by heading - 180-360
#define SAILOR_PARAM_SPEED2            20   
#define SAILOR_PARAM_SPEED2_E          4

// @pub 21;u8;3;flags;[0] = onStarboardTack,  [1] = tackLocked,  [2] = lastCrossTrackPositive
#define SAILOR_PARAM_FLAGS             21  
#define SAILOR_PARAM_FLAGS_E           5

// @pub 22;u8;1;wing;Wingsail flap position.. 1 = starboard wind, -1 = port wind
#define SAILOR_PARAM_WING              22   
#define SAILOR_PARAM_WING_E            6

#define SAILOR_PARAM_ENTRIES           7


static const char SAILOR_STR_SAILOR[] PROGMEM = "Sailor";

class SailorModule:  public DroneModule {
protected:
  boolean _starboardTack;  // true if stardboard, false if port
  boolean _tackLocked; // hyterisis control
  boolean _lastCrossTrackPositive; // hysterisis control

public:

  SailorModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  uint8_t polarIndexForAngle(float ang);
  uint8_t polarForAngle(float ang);

  void update();
  void loop();
};

#endif
