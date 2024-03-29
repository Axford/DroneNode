/*
@type PanTilt
@inherit Drone
@category      Output
@description Generate servo commands for a pan/tilt head to track a target (e.g. for camera or antenna trackers)

@guide >>>

<<<

@config >>>
PanTilt.new 21
  name "PanTilt"
  PID (f) 0.005 0.0 0.0001
  interval 50
  $target [@>20.8]
  $heading [@>6.11]
  .publish "target"
  .publish "heading"
  .publish "PID"
  .publish "pan"
.done
<<<

*/

#ifndef PAN_TILT_MODULE_H
#define PAN_TILT_MODULE_H

#include "../DroneModule.h"

// subs
// @sub 10;11;f;1;target;Target heading (e.g. from Nav module)
#define PAN_TILT_SUB_TARGET          10
#define PAN_TILT_SUB_TARGET_ADDR     11
#define PAN_TILT_SUB_TARGET_E        0

// @sub 12;13;f;1;Current heading (e.g. from Compass)
#define PAN_TILT_SUB_HEADING         12
#define PAN_TILT_SUB_HEADING_ADDR    13
#define PAN_TILT_SUB_HEADING_E       1

#define PAN_TILT_SUBS                2

// outputs

// @pub 14;f;3;w;PID;PID values (start with: 0.005 0.0 0.0001)
#define PAN_TILT_PARAM_PID           14
#define PAN_TILT_PARAM_PID_E         0

// @pub 15;f;2;w;limits;Limits of servo travel in degrees (default -90 90)
#define PAN_TILT_PARAM_LIMITS        15
#define PAN_TILT_PARAM_LIMITS_E      1

// @pub 16;f;1;r;pan;Pan output in range -1..1, to be fed to servo
#define PAN_TILT_PARAM_PAN           16
#define PAN_TILT_PARAM_PAN_E         2

#define PAN_TILT_PARAM_ENTRIES       3


static const char PAN_TILT_STR_PAN_TILT[] PROGMEM = "PanTilt";

class PanTiltModule:  public DroneModule {
protected:
  unsigned long _lastUpdate;
  float _lastTarget;
  float _iError;
  float _dError;
  float _lastError;
public:

  PanTiltModule(uint8_t id, DroneSystem* ds);
  
  //shortestSignedDistanceBetweenCircularValues
  static float getRotationDistance(float origin, float target);

  void update();
};

#endif
