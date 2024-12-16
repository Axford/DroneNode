/*
@type PanSerial
@description Generate serial commands for a slave pan controller module

Serial format is:
  ^ start char
  <ang>  (encoded 0..255 = 0..360)
  ! stop char

Serial port 2
Baud 9600

@guide >>>

<<<

@config >>>
PanSerial.new 21
  name "PanSerial"
  PID (f) 0.005 0.0 0.0001
  interval 50
  $target [@>20.8]
  $heading [@>6.11]
  .publish "target"
  .publish "heading"
  .publish "PID"
  .publish "pan"
.done
<<<

*/

#ifndef PAN_SERIAL_MODULE_H
#define PAN_SERIAL_MODULE_H

#include "../DroneModule.h"

// subs
// @sub 10;11;f;1;target;Target heading (e.g. from Nav module)
#define PAN_SERIAL_SUB_TARGET          10
#define PAN_SERIAL_SUB_TARGET_ADDR     11
#define PAN_SERIAL_SUB_TARGET_E        0

// @sub 12;13;f;1;Current heading (e.g. from Compass)
#define PAN_SERIAL_SUB_HEADING         12
#define PAN_SERIAL_SUB_HEADING_ADDR    13
#define PAN_SERIAL_SUB_HEADING_E       1

#define PAN_SERIAL_SUBS                2

// outputs

// @pub 14;f;3;w;PID;PID values (start with: 0.005 0.0 0.0001)
#define PAN_SERIAL_PARAM_PID           14
#define PAN_SERIAL_PARAM_PID_E         0

// @pub 15;u8;1;r;pan;Pan output in range 0..255 as sent over serial
#define PAN_SERIAL_PARAM_PAN           15
#define PAN_SERIAL_PARAM_PAN_E         1

#define PAN_SERIAL_PARAM_ENTRIES       2


static const char PAN_SERIAL_STR_PAN_SERIAL[] PROGMEM = "PanSerial";

class PanSerialModule:  public DroneModule {
protected:
  unsigned long _lastUpdate;
  float _lastTarget;
  float _iError;
  float _dError;
  float _lastError;
  Stream *_port;
public:

  PanSerialModule(uint8_t id, DroneSystem* ds);

  static float getRotationDistance(float origin, float target);

  void setup();

  void update();
};

#endif
