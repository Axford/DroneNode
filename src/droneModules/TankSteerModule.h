/*

Tank steering:
   * Sub to target heading
   * Sub to current heading
   * Sub to distance
   * Generate left and right motor speed values

*/
#ifndef TANK_STEER_MODULE_H
#define TANK_STEER_MODULE_H

#include "../DroneModule.h"

//pubs
#define TANK_STEER_PARAM_LEFT          8
#define TANK_STEER_PARAM_LEFT_E        0

#define TANK_STEER_PARAM_RIGHT         9
#define TANK_STEER_PARAM_RIGHT_E       1

#define TANK_STEER_PARAM_ENTRIES       2

// subs
#define TANK_STEER_SUB_TURN_RATE      10
#define TANK_STEER_SUB_TURN_RATE_ADDR 11
#define TANK_STEER_SUB_TURN_RATE_E    0

#define TANK_STEER_SUB_SPEED          12
#define TANK_STEER_SUB_SPEED_ADDR     13
#define TANK_STEER_SUB_SPEED_E        1

#define TANK_STEER_SUBS               2



static const char TANK_STEER_STR_TANK_STEER[] PROGMEM = "TankSteer";

class TankSteerModule:  public DroneModule {
protected:

public:

  TankSteerModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm);

  void update();
};

#endif
