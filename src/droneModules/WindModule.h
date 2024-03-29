
/*

@type          Wind
@inherits      I2CBase
@category      Input.Wind
@description   Manages a Wind speed (pulse count) and direction sensor (I2C AS5600)

@guide >>>
- Wind direction using I2C AS5600 sensor
- Wind speed from cup anemometor on an digital input with internal pullup
- Optional moving average with adjustable sample depth, 1 per interval
<<<

@config >>>
[Wind= 14]
  name= "Wind"
  interval= 100
  bus= 4
  $heading= @>8.11
  centre= -10
  samples=10
  publish= heading, direction, centre, wind

<<<
*/

#ifndef WIND_MODULE_H
#define WIND_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

//#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883_U.h>

#include <AS5600.h>

/*
@I2CAddress        0x36
@default addr = 54
*/ 
#define WIND_I2C_ADDRESS  0x36

// pubs
// @pub 10;f;1;r;direction;local wind direction
#define WIND_PARAM_DIRECTION       (I2CBASE_SUBCLASS_PARAM_START+0)  //10

// @pub 11;f;1;r;speed;wind speed
#define WIND_PARAM_SPEED           (I2CBASE_SUBCLASS_PARAM_START+1)  // 11

// @pub 12;u8;1;w;pins;Pin to use for anemometer interrupt signal
#define WIND_PARAM_PINS            (I2CBASE_SUBCLASS_PARAM_START+2)  // 12

// // @pub 13;f;1;w;centre;Calibration for local wind direction
#define WIND_PARAM_CENTRE          (I2CBASE_SUBCLASS_PARAM_START+3)  // 13

// // @pub 14;f;1;r;wind;World wind direction - combination of local direction and heading
#define WIND_PARAM_WIND            (I2CBASE_SUBCLASS_PARAM_START+4)  // 14

// @pub 15;u8;1;w;mode;Operation mode 0=standard, 1=inverted (upside down, e.g. for water direction sensor), 2=anemometer
#define WIND_PARAM_MODE            (I2CBASE_SUBCLASS_PARAM_START+5)  // 15

// @pub 18;u8;1;w;samples;Sample depth for moving average, 1 per interval (default 1, max 60)
#define WIND_PARAM_SAMPLES         (I2CBASE_SUBCLASS_PARAM_START+8)  // 18

// @pub 19;f;1;r;stats;Average variance relative to the moving average
#define WIND_PARAM_STATS           (I2CBASE_SUBCLASS_PARAM_START+9)  // 19

#define WIND_PARAM_DIRECTION_E     (I2CBASE_PARAM_ENTRIES+0)
#define WIND_PARAM_SPEED_E         (I2CBASE_PARAM_ENTRIES+1)
#define WIND_PARAM_PINS_E          (I2CBASE_PARAM_ENTRIES+2)
#define WIND_PARAM_CENTRE_E        (I2CBASE_PARAM_ENTRIES+3)
#define WIND_PARAM_WIND_E          (I2CBASE_PARAM_ENTRIES+4)
#define WIND_PARAM_MODE_E          (I2CBASE_PARAM_ENTRIES+5)
#define WIND_PARAM_SAMPLES_E       (I2CBASE_PARAM_ENTRIES+6)
#define WIND_PARAM_STATS_E         (I2CBASE_PARAM_ENTRIES+7)

#define WIND_PARAM_ENTRIES         (I2CBASE_PARAM_ENTRIES + 8)

// subs
// @sub 16;17;f;1;heading;Sub to compass heading to work out world wind direction
#define WIND_SUB_HEADING            (I2CBASE_SUBCLASS_PARAM_START+6)
#define WIND_SUB_HEADING_ADDR       (I2CBASE_SUBCLASS_PARAM_START+7)
#define WIND_SUB_HEADING_E          0

#define WIND_SUBS                   1

// strings
static const char WIND_STR_WIND[] PROGMEM = "Wind";

#define WIND_DIR_MAX_SAMPLES 250 // max 250 interval moving average
#define WIND_SPEED_SAMPLES   5 // moving average over 5 seconds

#define WIND_MODE_STANDARD   0
#define WIND_MODE_INVERTED   1
#define WIND_MODE_ANEMOMETER 2

// class
class WindModule:  public I2CBaseModule {
protected:
  AMS_5600 *_sensor;

  uint8_t _dirSample;

  float _speedSamples[WIND_SPEED_SAMPLES];
  uint8_t _speedSample;
  unsigned long _lastSpeedSampleTime;
public:

  WindModule(uint8_t id, DroneSystem* ds);
  ~WindModule();

  void doReset();

  static void ISR();

  void setup();
  void loop();


};

#endif
