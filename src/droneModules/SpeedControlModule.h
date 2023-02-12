/*

@type         SpeedControl
@inherits     Drone
@description  Convert a distance value (meters) into a speed control value (0..1)

@guide >>>
   <li> Sub to distance (from Nav)
   <li> Param for speed limits
   <li> Param for distance threshold at which to start slowing down
   <li> Generate speed output to feed TankSteer, etc
<<<
*/
#ifndef SPEED_CONTROL_MODULE_H
#define SPEED_CONTROL_MODULE_H

#include "../DroneModule.h"

// subs
// @sub 10;11;f;1;distance;Distance to target, typically Nav.distance
#define SPEED_CONTROL_SUB_DISTANCE       10
#define SPEED_CONTROL_SUB_DISTANCE_ADDR  11
#define SPEED_CONTROL_SUB_DISTANCE_E     0

#define SPEED_CONTROL_SUBS               1

// outputs
// @pub 12;f;2;limits;Min and max output speed range
#define SPEED_CONTROL_PARAM_LIMITS       12
#define SPEED_CONTROL_PARAM_LIMITS_E     0

// @pub 13;f;1;threshold;Distance in meters within which to throttle down to min limit
#define SPEED_CONTROL_PARAM_THRESHOLD    13
#define SPEED_CONTROL_PARAM_THRESHOLD_E  1

// @pub 14;f;1;speed;Output speed
#define SPEED_CONTROL_PARAM_SPEED        14
#define SPEED_CONTROL_PARAM_SPEED_E      2

#define SPEED_CONTROL_PARAM_ENTRIES      3


static const char SPEED_CONTROL_STR_SPEED_CONTROL[] PROGMEM = "SpeedControl";

class SpeedControlModule:  public DroneModule {
protected:

public:

  SpeedControlModule(uint8_t id, DroneSystem* ds);

  static float getRotationDistance(float origin, float target);

  void update();
};

#endif
