/*

Manages a PWM servo channel

*/
#ifndef SERVO_MODULE_H
#define SERVO_MODULE_H

#include "../DroneModule.h"
#include <ESP32Servo.h>


#define SERVO_SUB_POSITION         8
#define SERVO_SUB_POSITION_ADDR    9
#define SERVO_SUB_POSITION_E       0

#define SERVO_SUBS                 1


static const char SERVO_STR_SERVO[] PROGMEM = "Servo";

class ServoModule:  public DroneModule {
protected:
  uint8_t _pins[1];
  Servo _servo;
  float _limits[2];  // min, max
public:

  ServoModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);

  void loadConfiguration(JsonObject &obj);

  virtual void setup();
  virtual void loop();

  void update();

};

#endif
