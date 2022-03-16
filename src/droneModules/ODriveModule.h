/*

@type          ODrive
@inherits      Drone
@description   Manages an ODrive module via serial

@config >>>
ODrive.new 11
  name "ODrive"
  interval 100
  port 2
  limits -10 10
  invert 0
  $left [@>10.8]
  $right [@>10.9]

  // publish
  .publish "left"
  .publish "right"
.done
<<<

*/
#ifndef ODRIVE_MODULE_H
#define ODRIVE_MODULE_H

#include "../DroneModule.h"


// pubs

// @pub 12;f;2;limits;Min and max speed limits in range -1 to 1 (default -1 1)
#define ODRIVE_PARAM_LIMITS       12
#define ODRIVE_PARAM_LIMITS_E     0

// @pub 13;u8;1;port;Serial port (0..2, default: 2)
#define ODRIVE_PARAM_PORT         13
#define ODRIVE_PARAM_PORT_E       1

// @pub 14;u8;2;invert;Invert axes (0 = normal, 1 = invert)
#define ODRIVE_PARAM_INVERT       14
#define ODRIVE_PARAM_INVERT_E     2

// @pub 15;u8;1;invert;Switch axes (0=normal, 1=switched i.e. switch left and right motors)
#define ODRIVE_PARAM_SWITCH       15
#define ODRIVE_PARAM_SWITCH_E     3


#define ODRIVE_PARAM_ENTRIES      4

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



static const char ODRIVE_STR_ODRIVE[] PROGMEM = "ODrive";

class ODriveModule:  public DroneModule {
protected:
  Stream *_port;
public:

  ODriveModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void setPort(Stream *port);

  virtual void setup();

  void disable();

  void setVel(uint8_t axis, float v, boolean invert);

  void update();

};

#endif
