/*

@type          KiteController
@inherits      Drone
@category      Logic
@description   Generate left and right motor speeds from target course, current heading and distance to target

@guide >>>
For MVP

Use throttle input to determine relative position of the two drive motors

Params:
trim - offset applied to motors (added to right, subtracted from left)
limits - applied to throttle delta to drive motor position offset, -1 to 1 throttle is scaled to limits[0] to limits[1]

Inputs:
turnRate
Yaw
Pitch

Outputs:
Motor positions


<<<

@config >>>
[ KiteController = 8 ]
  name = KiteController
  mode = 1
  trim = 0
  limits = -1, 1
  $turnRate = @>TurnRate.analog
  $yaw = @>Yaw.analog
  $pitch = @>Analog.analog
  publish = mode, trim, limits
  publish = turnRate, yaw, pitch
  publish = left, right
<<<

*/
#ifndef KITE_CONTROLLER_MODULE_H
#define KITE_CONTROLLER_MODULE_H

#include "../DroneModule.h"

//pubs
// @pub 8;f;1;r;left;Left motor speed output in range -1..1
#define KITE_CONTROLLER_PARAM_LEFT          8
#define KITE_CONTROLLER_PARAM_LEFT_E        0

// @pub 9;f;1;r;right;Right motor speed output in range -1..1
#define KITE_CONTROLLER_PARAM_RIGHT         9
#define KITE_CONTROLLER_PARAM_RIGHT_E       1

// @pub 10;u8;1;w;mode;1=Automated (responds to speed and turnRate subs) or 0=Manual (ignores subs, but speed and turnRate values can be directly written to)
#define KITE_CONTROLLER_PARAM_MODE          10
#define KITE_CONTROLLER_PARAM_MODE_E        2

// @pub 11;f;1;w;trim;Applied to turnRate value to correct left/right imbalance in motor power
#define KITE_CONTROLLER_PARAM_TRIM          11
#define KITE_CONTROLLER_PARAM_TRIM_E        3
 
// @pub 12;f;2;w;limits;Min and max output speed range
#define KITE_CONTROLLER_PARAM_LIMITS        12
#define KITE_CONTROLLER_PARAM_LIMITS_E      4

// @pub 13;f;1;w;distance;Payout distance
#define KITE_CONTROLLER_PARAM_DISTANCE      13
#define KITE_CONTROLLER_PARAM_DISTANCE_E    5


#define KITE_CONTROLLER_PARAM_ENTRIES       6

// subs
// @sub 20;21;f;1;turnRate;Turn rate used to drive relative motor position and cause the kite to turn
#define KITE_CONTROLLER_SUB_TURN_RATE          20
#define KITE_CONTROLLER_SUB_TURN_RATE_ADDR     21
#define KITE_CONTROLLER_SUB_TURN_RATE_E        0

// @sub 22;23;f;1;yaw;Yaw from angle sensor
#define KITE_CONTROLLER_SUB_YAW                22
#define KITE_CONTROLLER_SUB_YAW_ADDR           23
#define KITE_CONTROLLER_SUB_YAW_E              1

// @sub 24;25;f;1;pitch;Pitch from angle sensor
#define KITE_CONTROLLER_SUB_PITCH              24
#define KITE_CONTROLLER_SUB_PITCH_ADDR         25
#define KITE_CONTROLLER_SUB_PITCH_E            2


#define KITE_CONTROLLER_SUBS                   3

// modes
#define KITE_CONTROLLER_MODE_MANUAL        0
#define KITE_CONTROLLER_MODE_AUTOMATIC     1


static const char KITE_CONTROLLER_STR_KITE_CONTROLLER[] PROGMEM = "KiteController";

class KiteControllerModule:  public DroneModule {
protected:
  unsigned long _lastUpdate;
  uint8_t _lastMode;

public:

  KiteControllerModule(uint8_t id, DroneSystem* ds);
  
  void loop();
};

#endif
