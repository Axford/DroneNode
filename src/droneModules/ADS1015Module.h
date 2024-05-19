/*

@type          ADS1015
@inherits      I2CBase
@category      Input
@description   Manages a Adafruit ADS1015 4-channel 12-bit ADC. Gain set so that raw 4095 = 4.095V

https://learn.adafruit.com/adafruit-4-channel-adc-breakouts/arduino-code

@config >>>
[ ADS1015 = 14 ]
  name = ADS1015
  bus = 4
  channels = 1
  limits = 0, 0.80566
  publish = value1, raw
<<<

*/
#ifndef ADS1015_MODULE_H
#define ADS1015_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "Adafruit_ADS1X15.h"

/*
@I2CAddress        0x48
@default addr = 72
*/ 
#define ADS1015_I2C_ADDRESS  0x49

// @pub 10;f;1;w;value1;ADS1015 value 1
#define ADS1015_PARAM_VALUE1        (I2CBASE_SUBCLASS_PARAM_START+0) // 10
// @pub 11;f;1;w;value2;ADS1015 value 2
#define ADS1015_PARAM_VALUE2        (I2CBASE_SUBCLASS_PARAM_START+1) // 11
// @pub 12;f;1;w;value3;ADS1015 value 3
#define ADS1015_PARAM_VALUE3        (I2CBASE_SUBCLASS_PARAM_START+2) // 12
// @pub 13;f;1;w;value4;ADS1015 value 4
#define ADS1015_PARAM_VALUE4        (I2CBASE_SUBCLASS_PARAM_START+3) // 13

// @pub 14;u8;1;w;channels;Number of ADC channels to use (1-4), default:1
#define ADS1015_PARAM_CHANNELS      (I2CBASE_SUBCLASS_PARAM_START+4) // 14

// @pub 15;u32;4;r;raw;Raw ADC readings 0..4095
#define ADS1015_PARAM_RAW           (I2CBASE_SUBCLASS_PARAM_START+5) // 15

// @pub 16;f;2;w;limits;Min and max values to map the raw readings into
#define ADS1015_PARAM_LIMITS        (I2CBASE_SUBCLASS_PARAM_START+6) // 16


#define ADS1015_PARAM_VALUE1_E      (I2CBASE_PARAM_ENTRIES+0)
#define ADS1015_PARAM_VALUE2_E      (I2CBASE_PARAM_ENTRIES+1)
#define ADS1015_PARAM_VALUE3_E      (I2CBASE_PARAM_ENTRIES+2)
#define ADS1015_PARAM_VALUE4_E      (I2CBASE_PARAM_ENTRIES+3)

#define ADS1015_PARAM_CHANNELS_E    (I2CBASE_PARAM_ENTRIES+4)
#define ADS1015_PARAM_RAW_E         (I2CBASE_PARAM_ENTRIES+5)
#define ADS1015_PARAM_LIMITS_E      (I2CBASE_PARAM_ENTRIES+6)


#define ADS1015_PARAM_ENTRIES       (I2CBASE_PARAM_ENTRIES + 7)


// strings
static const char ADS1015_STR_ADS1015[] PROGMEM = "ADS1015";

// class
class ADS1015Module:  public I2CBaseModule {
protected:
  Adafruit_ADS1X15* _sensor;
public:

  ADS1015Module(uint8_t id, DroneSystem* ds);
  ~ADS1015Module();

  void doReset();

  void setup();
  void loop();
};

#endif
