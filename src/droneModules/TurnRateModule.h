/*

Tank steering:
   * Sub to target heading
   * Sub to current heading
   * Tuning parameters (PID)
   * Generate turn rate

*/
#ifndef TURN_RATE_MODULE_H
#define TURN_RATE_MODULE_H

#include "../DroneModule.h"

// subs
#define TURN_RATE_SUB_TARGET          10
#define TURN_RATE_SUB_TARGET_ADDR     11
#define TURN_RATE_SUB_TARGET_E        0

#define TURN_RATE_SUB_HEADING         12
#define TURN_RATE_SUB_HEADING_ADDR    13
#define TURN_RATE_SUB_HEADING_E       1

#define TURN_RATE_SUB_PID             14
#define TURN_RATE_SUB_PID_ADDR        15
#define TURN_RATE_SUB_PID_E           2

#define TURN_RATE_SUBS                3

// outputs

#define TURN_RATE_PARAM_TURN_RATE       16
#define TURN_RATE_PARAM_TURN_RATE_E     0

#define TURN_RATE_PARAM_ENTRIES         1


static const char TURN_RATE_STR_TURN_RATE[] PROGMEM = "TurnRate";

class TurnRateModule:  public DroneModule {
protected:

public:

  TurnRateModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  //shortestSignedDistanceBetweenCircularValues
  static float getRotationDistance(float origin, float target);

  void update();
};

#endif
