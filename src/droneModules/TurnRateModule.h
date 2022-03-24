/*
@type TurnRate
@description Generate a turnRate command based on target vs current heading using a PID controller

@guide >>>
Tank steering:
   * Sub to target heading
   * Sub to current heading
   * Tuning parameters (PID)
   * Generate turn rate
<<<

@config >>>
TurnRate.new 8
  name "TurnRate"
  PID (f) 0.005 0.0 0.0001
  interval 50
  $target [@>22.14]
  $heading [@>50.8]
  .publish "target"
  .publish "heading"
  .publish "PID"
  .publish "turnRate"
.done
<<<

*/

#ifndef TURN_RATE_MODULE_H
#define TURN_RATE_MODULE_H

#include "../DroneModule.h"

// subs
// @sub 10;11;f;1;target;Target heading (e.g. from Nav module)
#define TURN_RATE_SUB_TARGET          10
#define TURN_RATE_SUB_TARGET_ADDR     11
#define TURN_RATE_SUB_TARGET_E        0

// @sub 12;13;f;1;Current heading (e.g. from Compass)
#define TURN_RATE_SUB_HEADING         12
#define TURN_RATE_SUB_HEADING_ADDR    13
#define TURN_RATE_SUB_HEADING_E       1

// @sub 14;15;f;3;PID values (start with: 0.005 0.0 0.0001)
#define TURN_RATE_SUB_PID             14
#define TURN_RATE_SUB_PID_ADDR        15
#define TURN_RATE_SUB_PID_E           2

#define TURN_RATE_SUBS                3

// outputs
// @pub 16;f;1;turnRate;turnRate output in range -1..1, where 1 is clockwise
#define TURN_RATE_PARAM_TURN_RATE       16
#define TURN_RATE_PARAM_TURN_RATE_E     0

#define TURN_RATE_PARAM_ENTRIES         1


static const char TURN_RATE_STR_TURN_RATE[] PROGMEM = "TurnRate";

class TurnRateModule:  public DroneModule {
protected:
  unsigned long _lastUpdate;
  float _iError;
  float _dError;
  float _lastError;
public:

  TurnRateModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  //shortestSignedDistanceBetweenCircularValues
  static float getRotationDistance(float origin, float target);

  //void update();

  void loop();
};

#endif
