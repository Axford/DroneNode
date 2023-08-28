/*

@type          Motor
@inherits      Drone
@description   Manages a Motor via an H Bridge module using PWM

@config >>>
[Motor = 11]
  name = "LeftMotor"
  interval = 50
  pins = OUT0_0, OUT0_1, DAC0_0
  PWMChannel= 15
  limits= -0.7, 0.7
  deadband= 0.3
  $speed = @>10.8
  publish = speed
<<<

*/
#ifndef MOTOR_MODULE_H
#define MOTOR_MODULE_H

#include "../DroneModule.h"


// pubs

// @pub 10;u8;3;pins;Pin connections - dependent on mode (e.g. A, B, EN for standard H-bridge)
#define MOTOR_PARAM_PINS         10
#define MOTOR_PARAM_PINS_E       0

// @pub 11;u8;1;PWMChannel;Which ESP32 PWM channel to use (default 15), mode 1 requires two channels and will use +1 as well
#define MOTOR_PARAM_PWMCHANNEL   11
#define MOTOR_PARAM_PWMCHANNEL_E 1

// @pub 12;f;2;limits;Min and max speed limits in range -1 to 1 (default -1 1)
#define MOTOR_PARAM_LIMITS       12
#define MOTOR_PARAM_LIMITS_E     2

// @pub 13;f;1;deadband;Input range within which to set the motor output to zero, interpreted abs(input).  Default 0.3
#define MOTOR_PARAM_DEADBAND     13
#define MOTOR_PARAM_DEADBAND_E   3

// @pub 14;u8;1;mode;Operational mode. 0: standard H-bridge with A, B, and PWM-EN. 1: BTS7960 with PWM F & R.  2: Cytron with PWM speed + Dir
#define MOTOR_PARAM_MODE         14
#define MOTOR_PARAM_MODE_E       4

// @pub 15;u8;1;invert;Invert output direction (0=normal, 1=inverted)
#define MOTOR_PARAM_INVERT       15
#define MOTOR_PARAM_INVERT_E     5

#define MOTOR_PARAM_ENTRIES      6

// subs
// @sub 8;9;f;1;speed;Desired Motor speed in range -1 to 1
#define MOTOR_SUB_SPEED         8
#define MOTOR_SUB_SPEED_ADDR    9
#define MOTOR_SUB_SPEED_E       0

#define MOTOR_SUBS              1

// indices to the _pins array
// mode 0
#define MOTOR_PIN_A   0
#define MOTOR_PIN_B   1
#define MOTOR_PIN_EN  2
// mode 1
#define MOTOR_PIN_F   0
#define MOTOR_PIN_R   1
// mode 2
#define MOTOR_PIN_PWM   0
#define MOTOR_PIN_DIR   1


static const char MOTOR_STR_MOTOR[] PROGMEM = "Motor";

class MotorModule:  public DroneModule {
protected:

public:

  MotorModule(uint8_t id, DroneSystem* ds);

  virtual void setup();

  boolean requestMotorPins(uint8_t num);

  void setupMode0();
  void setupMode1();
  void setupMode2();

  void disable();

  void update();

};

#endif
