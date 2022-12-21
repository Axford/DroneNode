/*

@type TankSteer
@inherits Drone
@description Generate left and right motor speeds from target course, current heading and distance to target

@guide >>>

<<<

@config >>>

<<<

*/
#ifndef TANK_STEER_MODULE_H
#define TANK_STEER_MODULE_H

#include "../DroneModule.h"

//pubs
// @pub 8;f;1;left;Left motor speed output in range -1..1
#define TANK_STEER_PARAM_LEFT          8
#define TANK_STEER_PARAM_LEFT_E        0

// @pub 9;f;1;right;Right motor speed output in range -1..1
#define TANK_STEER_PARAM_RIGHT         9
#define TANK_STEER_PARAM_RIGHT_E       1

// @pub 16;u8;1;mode;1=Automated (responds to speed and turnRate subs) or 0=Manual (ignores subs, but speed and turnRate values can be directly written to)
#define TANK_STEER_PARAM_MODE          10
#define TANK_STEER_PARAM_MODE_E        2

// @pub 14;f;1;trim;Applied to turnRate value to correct left/right imbalance in motor power
#define TANK_STEER_PARAM_TRIM          11
#define TANK_STEER_PARAM_TRIM_E        3

// @pub 12;f;3;PID;PID settings for turnRate control
#define TANK_STEER_PARAM_PID           12
#define TANK_STEER_PARAM_PID_E         4
 
// @pub 12;f;2;limits;Min and max output speed range
#define TANK_STEER_PARAM_LIMITS        13
#define TANK_STEER_PARAM_LIMITS_E      5

// @pub 13;f;1;threshold;Distance in meters within which to throttle down to min limit
#define TANK_STEER_PARAM_THRESHOLD     14
#define TANK_STEER_PARAM_THRESHOLD_E   6

#define TANK_STEER_PARAM_ENTRIES       7

// subs
// @sub 20;21;f;1;target;Target heading (e.g. from Nav module)
#define TANK_STEER_SUB_TARGET          20
#define TANK_STEER_SUB_TARGET_ADDR     21
#define TANK_STEER_SUB_TARGET_E        0

// @sub 22;23;f;1;heading;Current heading (e.g. from Compass)
#define TANK_STEER_SUB_HEADING         22
#define TANK_STEER_SUB_HEADING_ADDR    23
#define TANK_STEER_SUB_HEADING_E       1

// @sub 24;25;f;1;distance;Distance to target, typically Nav.distance
#define TANK_STEER_SUB_DISTANCE       24
#define TANK_STEER_SUB_DISTANCE_ADDR  25
#define TANK_STEER_SUB_DISTANCE_E     2


#define TANK_STEER_SUBS               3

// modes
#define TANK_STEER_MODE_MANUAL        0
#define TANK_STEER_MODE_AUTOMATIC     1


static const char TANK_STEER_STR_TANK_STEER[] PROGMEM = "TankSteer";

class TankSteerModule:  public DroneModule {
protected:
  unsigned long _lastUpdate;
  float _iError;
  float _dError;
  float _lastError;
  float _lastHeading;
  uint8_t _lastMode;

public:

  TankSteerModule(uint8_t id, DroneSystem* ds);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void update();
};

#endif
