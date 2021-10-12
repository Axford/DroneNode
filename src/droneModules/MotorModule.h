/*

Manages a PWM MOTOR channel

*/
#ifndef MOTOR_MODULE_H
#define MOTOR_MODULE_H

#include "../DroneModule.h"


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
  float _deadband;  // width of deadband (measured from zero, do actually double this value)
  uint8_t _PWMChannel;
  uint8_t _pins[3];
  float _limits[2];  // min, max

public:

  MotorModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm);

  void loadConfiguration(JsonObject &obj);

  virtual void setup();

  void disable();

  void update();

};

#endif
