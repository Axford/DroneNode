/*

@type           Servo
@inherits       Drone
@description    Manages a PWM servo channel

@config >>>
[Servo= 13]
  position= 0
  pins= 4
  status= 1
  name= "Rudder"
  map= 50, 80, 100, 130
  centre= 8
  $position= @>11.16 
  interval= 100
  publish= map, centre, position, output
<<<


*/
#ifndef SERVO_MODULE_H
#define SERVO_MODULE_H

#include "../DroneModule.h"
#include <ESP32Servo.h>

// params
// @pub 10;u8;1;pins;Servo output pin, see <a href="pins.html">Pin Mappings</a>
#define SERVO_PARAM_PINS         10
#define SERVO_PARAM_PINS_E       0

// @pub 11;f;1;limits;Movement rate limit in degrees/second
#define SERVO_PARAM_LIMITS       11
#define SERVO_PARAM_LIMITS_E     1

// @pub 12;f;4;map;Bezier curve control points to map input (-1..1) to output (0..180)
#define SERVO_PARAM_MAP          12
#define SERVO_PARAM_MAP_E        2

// @pub 13;f;1;centre;Centre adjustment, applied after map
#define SERVO_PARAM_CENTRE       13
#define SERVO_PARAM_CENTRE_E     3

// @pub 14;f;1;output;The raw angle sent to the Servo PWM controller (0..180)
#define SERVO_PARAM_OUTPUT       14
#define SERVO_PARAM_OUTPUT_E     4

#define SERVO_PARAM_ENTRIES      5


// subs
// @sub 8;9;f;1;position;The input position value (-1..1)
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
