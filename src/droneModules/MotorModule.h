/*

@type          Motor
@inherits      Drone
@description   Manages a Motor via an H Bridge module using PWM

@config >>>
Motor.new 11
  name "LeftMotor"
  interval 50
  pins OUT0_0 OUT0_1 DAC0_0
  PWMChannel 15
  limits -0.7 0.7
  deadband 0.3
  $speed [@>10.8]

  // publish
  .publish "speed"
.done
<<<

*/
#ifndef MOTOR_MODULE_H
#define MOTOR_MODULE_H

#include "../DroneModule.h"


// pubs

// @pub 10;u8;3;pins;Pin connections for the H Bridge (A, B, EN)
#define MOTOR_PARAM_PINS         10
#define MOTOR_PARAM_PINS_E       0

// @pub 11;u8;1;PWMChannel;Which ESP32 PWM channel to use (default 15)
#define MOTOR_PARAM_PWMCHANNEL   11
#define MOTOR_PARAM_PWMCHANNEL_E 1

// @pub 12;f;2;limits;Min and max speed limits in range -1 to 1 (default -1 1)
#define MOTOR_PARAM_LIMITS       12
#define MOTOR_PARAM_LIMITS_E     2

// @pub 13;f;1;deadband;Input range within which to set the motor output to zero, interpreted abs(input).  Default 0.3
#define MOTOR_PARAM_DEADBAND     13
#define MOTOR_PARAM_DEADBAND_E   3

#define MOTOR_PARAM_ENTRIES      4

// subs
// @sub 8;9;f;1;speed;Desired Motor speed in range -1 to 1
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

  MotorModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);


  virtual void setup();

  void disable();

  void update();

};

#endif
