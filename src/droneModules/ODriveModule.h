/*

@type          ODrive
@inherits      Drone
@category      Output
@description   Manages an ODrive module via serial

@config >>>
[ ODrive = 11]
  name = "ODrive"
  interval = 100
  port = 2
  limits = -10, 10
  invert = 0
  $left = @>10.8
  $right = @>10.9
  publish = left, right, limits, invert


[ ODrive = 11]
  name = "ODrive"
  mode = 1
  interval = 100
  port = 2
  limits = 1, 1
  invert = 0
  $left = @>10.8
  $right = @>10.9
  publish = left, right, limits, invert
<<<

*/
#ifndef ODRIVE_MODULE_H
#define ODRIVE_MODULE_H

#include "../DroneModule.h"


// pubs

// @pub 12;f;2;w;limits;In velocity mode these are Min and max speed limits in range -1 to 1 (default -1 1). In position mode these are limits for Velocity in turns/s [0] and Torque in Nm [1]
#define ODRIVE_PARAM_LIMITS       12
#define ODRIVE_PARAM_LIMITS_E     0

// @pub 13;u8;1;w;port;Serial port (0..2, default: 2)
#define ODRIVE_PARAM_PORT         13
#define ODRIVE_PARAM_PORT_E       1

// @pub 14;u8;2;w;invert;Invert axes (0 = normal, 1 = invert)
#define ODRIVE_PARAM_INVERT       14
#define ODRIVE_PARAM_INVERT_E     2

// @pub 15;u8;1;w;switch;Switch axes (0=normal, 1=switched i.e. switch left and right motors)
#define ODRIVE_PARAM_SWITCH       15
#define ODRIVE_PARAM_SWITCH_E     3

// @pub 16;u8;1;w;mode;Mode: 0=velocity control, 1=position control (default: 0)
#define ODRIVE_PARAM_MODE         16
#define ODRIVE_PARAM_MODE_E       4

#define ODRIVE_PARAM_ENTRIES      5

// subs
// @sub 8;9;f;1;left;Desired left motor speed in range -1 to 1
#define ODRIVE_SUB_LEFT         8
#define ODRIVE_SUB_LEFT_ADDR    9
#define ODRIVE_SUB_LEFT_E       0

// @sub 10;11;f;1;right;Desired right motor speed in range -1 to 1
#define ODRIVE_SUB_RIGHT        10
#define ODRIVE_SUB_RIGHT_ADDR   11
#define ODRIVE_SUB_RIGHT_E      1

#define ODRIVE_SUBS             2


#define ODRIVE_MODE_VELOCITY_CONTROL   0
#define ODRIVE_MODE_POSITION_CONTROL   1

static const char ODRIVE_STR_ODRIVE[] PROGMEM = "ODrive";

class ODriveModule:  public DroneModule {
protected:
  Stream *_port;
public:

  ODriveModule(uint8_t id, DroneSystem* ds);

  void setPort(Stream *port);

  virtual void setup();

  void disable();

  void setVel(uint8_t axis, float v, boolean invert);

  void update();
  void loop();
};

#endif
