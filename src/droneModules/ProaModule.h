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

// subs
#define PROA_SUB_TARGET              8
#define PROA_SUB_TARGET_ADDR         9
#define PROA_SUB_TARGET_E            0

// current compass heading
#define PROA_SUB_HEADING             10
#define PROA_SUB_HEADING_ADDR        11
#define PROA_SUB_HEADING_E           1

// sub to global wind direction
#define PROA_SUB_WIND                12
#define PROA_SUB_WIND_ADDR           13
#define PROA_SUB_WIND_E              2

#define PROA_SUB_CROSSTRACK          14
#define PROA_SUB_CROSSTRACK_ADDR     15
#define PROA_SUB_CROSSTRACK_E        3

#define PROA_SUBS                    4


//pubs
#define PROA_PARAM_COURSE            16
#define PROA_PARAM_COURSE_E          0

// wing servo angle (-1 .. 1)
#define PROA_PARAM_WING              17
#define PROA_PARAM_WING_E            1

#define PROA_PARAM_POLAR             18
#define PROA_PARAM_POLAR_E           2

#define PROA_PARAM_SPEED             19   // polar plot of estimated speeds by heading - 0-180
#define PROA_PARAM_SPEED_E           3

#define PROA_PARAM_SPEED2            20   // polar plot of estimated speeds by heading - 180-360
#define PROA_PARAM_SPEED2_E          4

#define PROA_PARAM_FLAGS             21   //
#define PROA_PARAM_FLAGS_E           5

// left pontoon servo angle (-1 .. 1)
#define PROA_PARAM_LEFT              22
#define PROA_PARAM_LEFT_E            6

// right pontoon servo angle (-1 .. 1)
#define PROA_PARAM_RIGHT             23
#define PROA_PARAM_RIGHT_E           7

#define PROA_PARAM_ENTRIES           8


static const char PROA_STR_PROA[] PROGMEM = "Proa";

class ProaModule:  public DroneModule {
protected:
  boolean _starboardTack;  // true if stardboard, false if port
  boolean _tackLocked; // hyterisis control
  boolean _lastCrossTrackPositive; // hysterisis control
  float _wingAOA;  // angle of attack

public:

  ProaModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  uint8_t polarIndexForAngle(float ang);
  uint8_t polarForAngle(float ang);

  void update();
  void loop();
};

#endif
