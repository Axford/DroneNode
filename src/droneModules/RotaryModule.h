/*

@type          Rotary
@inherits      I2CBase
@category      Input
@description   Manages a Adafruit NeoRotary quad encoder input module.  Initial values can be set in config.

@config >>>
[ ROTARY = 14 ]
  name = ROTARY
  bus = 4
  map = 1, 2, 10, 0.1
  input1 = 20
  publish = input1, input2, input3, input4, map
<<<

*/
#ifndef ROTARY_MODULE_H
#define ROTARY_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "Adafruit_seesaw.h"

/*
@I2CAddress        0x49
@default addr = 73
*/ 
#define ROTARY_I2C_ADDRESS  0x49

// @pub 10;f;1;w;input1;Rotary input 1
#define ROTARY_PARAM_INPUT1        (I2CBASE_SUBCLASS_PARAM_START+0) // 10
// @pub 11;f;1;w;input2;Rotary input 2
#define ROTARY_PARAM_INPUT2        (I2CBASE_SUBCLASS_PARAM_START+1) // 11
// @pub 12;f;1;w;input3;Rotary input 3
#define ROTARY_PARAM_INPUT3        (I2CBASE_SUBCLASS_PARAM_START+2) // 12
// @pub 13;f;1;w;input4;Rotary input 4
#define ROTARY_PARAM_INPUT4        (I2CBASE_SUBCLASS_PARAM_START+3) // 13

// @pub 14;f;4;w;map;Values to map integer rotary steps to output floats, default: [1,1,1,1]
#define ROTARY_PARAM_MAP           (I2CBASE_SUBCLASS_PARAM_START+4) // 14

// @pub 15;f;4;w;min;Minimum values for each input, default: [-100,-100,-100,-100]
#define ROTARY_PARAM_MIN           (I2CBASE_SUBCLASS_PARAM_START+5) // 15

// @pub 16;f;4;w;max;Maximum values for each input, default: [100,100,100,100]
#define ROTARY_PARAM_MAX           (I2CBASE_SUBCLASS_PARAM_START+6) // 16


#define ROTARY_PARAM_INPUT1_E      (I2CBASE_PARAM_ENTRIES+0)
#define ROTARY_PARAM_INPUT2_E      (I2CBASE_PARAM_ENTRIES+1)
#define ROTARY_PARAM_INPUT3_E      (I2CBASE_PARAM_ENTRIES+2)
#define ROTARY_PARAM_INPUT4_E      (I2CBASE_PARAM_ENTRIES+3)

#define ROTARY_PARAM_MAP_E         (I2CBASE_PARAM_ENTRIES+4)

#define ROTARY_PARAM_MIN_E         (I2CBASE_PARAM_ENTRIES+5)
#define ROTARY_PARAM_MAX_E         (I2CBASE_PARAM_ENTRIES+6)


#define ROTARY_PARAM_ENTRIES       (I2CBASE_PARAM_ENTRIES + 7)


// strings
static const char ROTARY_STR_ROTARY[] PROGMEM = "Rotary";

// class
class RotaryModule:  public I2CBaseModule {
protected:
  Adafruit_seesaw* _sensor;
  int32_t _encPositions[4];
  int32_t _encOffsets[4];  // to allow for default values
public:

  RotaryModule(uint8_t id, DroneSystem* ds);
  ~RotaryModule();

  void doReset();

  void setup();
  void loop();
};

#endif
