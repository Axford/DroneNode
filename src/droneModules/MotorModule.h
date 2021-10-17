/*

Manages a PWM MOTOR channel

*/
#ifndef MOTOR_MODULE_H
#define MOTOR_MODULE_H

#include "../DroneModule.h"


// params

#define MOTOR_PARAM_PINS         10
#define MOTOR_PARAM_PINS_E       0

#define MOTOR_PARAM_PWMCHANNEL   11
#define MOTOR_PARAM_PWMCHANNEL_E 1

#define MOTOR_PARAM_LIMITS       12
#define MOTOR_PARAM_LIMITS_E     2

#define MOTOR_PARAM_DEADBAND     13
#define MOTOR_PARAM_DEADBAND_E   3

#define MOTOR_PARAM_ENTRIES      4

// subs
#define MOTOR_SUB_SPEED         8
#define MOTOR_SUB_SPEED_ADDR    9
#define MOTOR_SUB_SPEED_E       0

#define MOTOR_SUBS              1

// indices to the _pins array
#define MOTOR_PIN_A   0
#define MOTOR_PIN_B   1
#define MOTOR_PIN_EN  2


static const char MOTOR_STR_MOTOR[] PROGMEM = "Motor";

class MotorModule:  public DroneModule {
protected:
  
public:

  MotorModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);


  void loadConfiguration(JsonObject &obj);

  virtual void setup();

  void disable();

  void update();

};

#endif
