/*

Manages a PWM servo channel

*/
#ifndef SERVO_MODULE_H
#define SERVO_MODULE_H

#include "../DroneModule.h"
#include <ESP32Servo.h>

// params

#define SERVO_PARAM_PINS         10
#define SERVO_PARAM_PINS_E       0

#define SERVO_PARAM_LIMITS       11
#define SERVO_PARAM_LIMITS_E     1

#define SERVO_PARAM_MAP          12
#define SERVO_PARAM_MAP_E        2

#define SERVO_PARAM_CENTRE       13
#define SERVO_PARAM_CENTRE_E     3

#define SERVO_PARAM_OUTPUT       14
#define SERVO_PARAM_OUTPUT_E     4

#define SERVO_PARAM_ENTRIES      5


// subs
#define SERVO_SUB_POSITION         8
#define SERVO_SUB_POSITION_ADDR    9
#define SERVO_SUB_POSITION_E       0

#define SERVO_SUBS                 1


static const char SERVO_STR_SERVO[] PROGMEM = "Servo";

class ServoModule:  public DroneModule {
protected:
  //uint8_t _pins[1];
  float _targetPos;
  float _startPos;
  float _currentPos;
  Servo _servo;
  unsigned long _startTime;
  //float _limits[2];  // min, max
public:

  ServoModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void setup();
  void loop();

  void update();

};

#endif
