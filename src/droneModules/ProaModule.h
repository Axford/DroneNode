/*

Proa controller

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
*/

#ifndef PROA_MODULE_H
#define PROA_MODULE_H

#include "../DroneModule.h"

// -------  subs ---------------------
// target heading (from nav)
#define PROA_SUB_TARGET              8
#define PROA_SUB_TARGET_ADDR         9
#define PROA_SUB_TARGET_E            0

// current compass heading (from compass)
#define PROA_SUB_HEADING             10
#define PROA_SUB_HEADING_ADDR        11
#define PROA_SUB_HEADING_E           1

// global wind direction (from wind vane)
#define PROA_SUB_WIND                12
#define PROA_SUB_WIND_ADDR           13
#define PROA_SUB_WIND_E              2

// crosstrack error (from nav)
#define PROA_SUB_CROSSTRACK          14
#define PROA_SUB_CROSSTRACK_ADDR     15
#define PROA_SUB_CROSSTRACK_E        3

// course over water (from water vane sensor)
#define PROA_SUB_COW              24
#define PROA_SUB_COW_ADDR         25
#define PROA_SUB_COW_E            4

//

#define PROA_SUBS                    5


// -------  pubs ---------------------
// current course
#define PROA_PARAM_COURSE            16
#define PROA_PARAM_COURSE_E          0

// wing servo angle (-1 .. 1)
#define PROA_PARAM_WING              17
#define PROA_PARAM_WING_E            1

// polar configuration
#define PROA_PARAM_POLAR             18
#define PROA_PARAM_POLAR_E           2

// polar plot of estimated speeds by heading - 0-180
#define PROA_PARAM_SPEED             19
#define PROA_PARAM_SPEED_E           3

// polar plot of estimated speeds by heading - 180-360
#define PROA_PARAM_SPEED2            20
#define PROA_PARAM_SPEED2_E          4

// status flags
#define PROA_PARAM_FLAGS             21
#define PROA_PARAM_FLAGS_E           5

// left pontoon servo angle (-1 .. 1)
#define PROA_PARAM_LEFT              22
#define PROA_PARAM_LEFT_E            6

// right pontoon servo angle (-1 .. 1)
#define PROA_PARAM_RIGHT             23
#define PROA_PARAM_RIGHT_E           7

// PID values for pontoon control
#define PROA_PARAM_PID               26
#define PROA_PARAM_PID_E             8

// Angle of attack
#define PROA_PARAM_AOA               27
#define PROA_PARAM_AOA_E             9

// angle to offset frame relative to target heading to orient wing correctly
#define PROA_PARAM_OFFSET            28
#define PROA_PARAM_OFFSET_E          10

// @pub 29;u8;1;mode;Operational mode: 0=passive for setup, 1=active
#define PROA_PARAM_MODE              29
#define PROA_PARAM_MODE_E            11

// @pub 30;f;4;debug;Debug parameters
#define PROA_PARAM_DEBUG             30
#define PROA_PARAM_DEBUG_E           12

#define PROA_PARAM_ENTRIES           13


// control modes - dictates wing and pontoon control logic
#define PROA_CONTROL_MODE_NORMAL     0  // orient wing for optimal AOA
#define PROA_CONTROL_MODE_BRAKE      1  // orient wing for air brake assisted turn
#define PROA_CONTROL_MODE_RUN        2  // orient wing for run


static const char PROA_STR_PROA[] PROGMEM = "Proa";

class ProaModule:  public DroneModule {
protected:
  boolean _starboardTack;  // true if stardboard, false if port
  boolean _tackLocked; // hyterisis control
  boolean _lastCrossTrackPositive; // hysterisis control

public:

  ProaModule(uint8_t id, DroneSystem* ds);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  uint8_t polarIndexForAngle(float ang);
  uint8_t polarForAngle(float ang);

  void update();
  void loop();
};

#endif
