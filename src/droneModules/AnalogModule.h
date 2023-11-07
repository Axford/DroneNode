/*

@type          Analog
@inherits      Drone
@category      Input
@description   Manages a simple Analog sensor connected to an analog input pin

@guide >>>
Reads an analog value on specified pin (range 0..4095) and maps into the range limits[min..max]
<<<

@config >>>
[Analog = 5]
  pins = 34
  limits = 0, 10
  publish = raw, analog
<<<

*/
#ifndef ANALOG_MODULE_H
#define ANALOG_MODULE_H

#include "../DroneModule.h"


// pubs

// @pub 10;u8;1;w;pins;Pin for the analog input
#define ANALOG_PARAM_PINS         10
#define ANALOG_PARAM_PINS_E       0

// @pub 11;u32;1;r;raw;Raw analog reading (0..4095)
#define ANALOG_PARAM_RAW          11
#define ANALOG_PARAM_RAW_E        1

// @pub 12;f;2;w;limits;Min and max values to map the raw reading into
#define ANALOG_PARAM_LIMITS       12
#define ANALOG_PARAM_LIMITS_E     2

// @pub 13;f;1;r;analog;Analog value after mapping into min/max range
#define ANALOG_PARAM_ANALOG       13
#define ANALOG_PARAM_ANALOG_E     3

#define ANALOG_PARAM_ENTRIES      4

// subs
#define ANALOG_SUBS               0


static const char ANALOG_STR_ANALOG[] PROGMEM = "Analog";

class AnalogModule:  public DroneModule {
protected:
public:

  AnalogModule(uint8_t id, DroneSystem* ds);
  
  virtual void setup();

  void loop();

};

#endif
