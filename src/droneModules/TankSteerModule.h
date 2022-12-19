/*

@type TankSteer
@inherits Drone
@description Generate left and right motor speeds from speed and turnRate inputs

@guide >>>
Tank steering:
   * Sub to target heading
   * Sub to current heading
   * Sub to distance
   * Generate left and right motor speed values
<<<

@config >>>
TankSteer.new 10
  name "TankSteer"
  interval 50
  trim 0
  $speed [@>9.14]
  $turnRate [@>8.16]
  .publish "left"
  .publish "right"
  .publish "turnRate"
  .publish "speed"
  .publish "trim"
.done
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
#define TANK_STEER_PARAM_MODE          16
#define TANK_STEER_PARAM_MODE_E        2

#define TANK_STEER_PARAM_ENTRIES       3

// subs
// @sub 10;11;f;1;turnRate;Rate of turn in range -1..1 where positive values are clockwise
#define TANK_STEER_SUB_TURN_RATE      10
#define TANK_STEER_SUB_TURN_RATE_ADDR 11
#define TANK_STEER_SUB_TURN_RATE_E    0

// @sub 12;13;f;1;speed;Desired speed in range -1 to 1
#define TANK_STEER_SUB_SPEED          12
#define TANK_STEER_SUB_SPEED_ADDR     13
#define TANK_STEER_SUB_SPEED_E        1

// @sub 14;15;f;1;trim;Applied to turnRate value
#define TANK_STEER_SUB_TRIM           14
#define TANK_STEER_SUB_TRIM_ADDR      15
#define TANK_STEER_SUB_TRIM_E         2

#define TANK_STEER_SUBS               3

// modes
#define TANK_STEER_MODE_MANUAL        0
#define TANK_STEER_MODE_AUTOMATIC     1


static const char TANK_STEER_STR_TANK_STEER[] PROGMEM = "TankSteer";

class TankSteerModule:  public DroneModule {
protected:

public:

  TankSteerModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void update();
};

#endif
