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

// @pub 21;u8;5;flags;[0] = state, [1] = tack state,  [2] = gybe state, [3] = course wind packed into 0..255
#define SAILOR_PARAM_FLAGS             21  
#define SAILOR_PARAM_FLAGS_E           5

// @pub 22;u8;1;wing;Wingsail flap position.. 1 = starboard wind, -1 = port wind
#define SAILOR_PARAM_WING              22   
#define SAILOR_PARAM_WING_E            6

// @pub 23;f;3;PID;PID values for rudder control (start with: 0.005 0.0 0.0)
#define SAILOR_PARAM_PID               23
#define SAILOR_PARAM_PID_E             7

// @pub 24;f;1;rudder;Rudder output to feed a servo (-1..1)
#define SAILOR_PARAM_RUDDER            24
#define SAILOR_PARAM_RUDDER_E          8

// @pub 25;f;1;threshold;Error threshold to trigger the gybe timer (default 20 degrees)
#define SAILOR_PARAM_THRESHOLD         25
#define SAILOR_PARAM_THRESHOLD_E       9

// @pub 26;f;1;timeout;Timeout duration in seconds after which to initiate the gybe (default 10s)
#define SAILOR_PARAM_TIMEOUT           26
#define SAILOR_PARAM_TIMEOUT_E         10

// @pub 27;u8;1;mode;Mode (0=normal, 1=gybe only)
#define SAILOR_PARAM_MODE              27
#define SAILOR_PARAM_MODE_E            11


#define SAILOR_PARAM_ENTRIES           12

#define SAILOR_FLAG_STATE              0
#define SAILOR_FLAG_TACK               1
#define SAILOR_FLAG_GYBE               2
#define SAILOR_FLAG_COURSE_WIND        3
#define SAILOR_FLAG_CROSS_THE_WIND     4

#define SAILOR_GYBE_NORMAL             0
#define SAILOR_GYBE_POTENTIAL          1
#define SAILOR_GYBE_GYBING             2

#define SAILOR_MODE_NORMAL             0
#define SAILOR_MODE_GYBE_ONLY          1

#define SAILOR_TACK_UNDEFINED          0
#define SAILOR_TACK_STARBOARD          1
#define SAILOR_TACK_PORT               2

#define SAILOR_STATE_PLANNING          0  // when a new course is required
#define SAILOR_STATE_COURSE_SET        1  // when a new course has been selected
#define SAILOR_STATE_COURSE_UNDERWAY   2  // when the abs(crosstrack) distance falls below 0.8 for some hysterisis on constant replanning near the edge of the corridor



static const char SAILOR_STR_SAILOR[] PROGMEM = "Sailor";

class SailorModule:  public DroneModule {
protected:
  float _courseWind;  // what direction was the wind when we selected the course
  float _courseTarget;  // what direction was the target when we selected the course

  // for rudder control - borrow from TurnRate module
  unsigned long _lastUpdate;
  float _iError;
  float _dError;
  float _lastError;
  float _lastHeading;

  uint32_t _gybeTimerStart; // if we are in potential gybe conditions, when did the timer start
  boolean _positiveError;  // did the timer start whilst the error is positive

  uint32_t _stallTimerStart;
  boolean _potentialStall;

public:

  SailorModule(uint8_t id, DroneSystem* ds);

  uint8_t polarIndexForAngle(float ang);
  uint8_t polarForAngle(float ang);

  void update();
  void loop();
};

#endif
