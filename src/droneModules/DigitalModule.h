/*

@type          Digital
@inherits      Drone
@category      Input
@description   Manages a simple digital IO input on a specified pin

@guide >>>
Reads a digital value on specified pin and maps into the values specified in limits[low, high]
<<<

@config >>>
[Digital = 5]
  pins = 32
  limits = 0, 1
  publish = input
<<<

*/
#ifndef DIGITAL_MODULE_H
#define DIGITAL_MODULE_H

#include "../DroneModule.h"


// pubs

// @pub 10;u8;1;w;pins;Pin for the digital input
#define DIGITAL_PARAM_PINS          10
#define DIGITAL_PARAM_PINS_E        0

// @pub 12;f;2;w;limits;Min and max values to map the digital value to (low maps to [0], high maps to [1])
#define DIGITAL_PARAM_LIMITS        11
#define DIGITAL_PARAM_LIMITS_E      1

// @pub 13;f;1;r;input;Digital input value after remapping based on limits
#define DIGITAL_PARAM_INPUT         12
#define DIGITAL_PARAM_INPUT_E       2

#define DIGITAL_PARAM_ENTRIES       3

// subs
#define DIGITAL_SUBS               0


static const char DIGITAL_STR_DIGITAL[] PROGMEM = "Digital";

class DigitalModule:  public DroneModule {
protected:
public:

  DigitalModule(uint8_t id, DroneSystem* ds);
  
  virtual void setup();

  void loop();

};

#endif
