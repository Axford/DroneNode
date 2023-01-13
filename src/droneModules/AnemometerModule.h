
/*

@type          Anemometer
@inherits      Drone
@description   Manages an Anemometer.

@guide >>>
- Anemometer speed from cup anemometor on an digital input with internal pullup
- Uses internal 5sec moving average on samples
- Conversion is 1.25 revs per m/s
<<<

@config >>>
[Anemometer= 14]
  name= "Anemometer"
  interval= 100
  pins = 32
  publish = speed

<<<
*/

#ifndef ANEMOMETER_MODULE_H
#define ANEMOMETER_MODULE_H

#include "../DroneModule.h"


// @pub 11;f;1;speed;wind speed in Knots
#define ANEMOMETER_PARAM_SPEED           10

// @pub 12;u8;1;pins;Pin to use for anemometer interrupt signal
#define ANEMOMETER_PARAM_PINS            11


#define ANEMOMETER_PARAM_SPEED_E         0
#define ANEMOMETER_PARAM_PINS_E          1

#define ANEMOMETER_PARAM_ENTRIES         2

// subs

#define ANEMOMETER_SUBS                   0

// strings
static const char ANEMOMETER_STR_ANEMOMETER[] PROGMEM = "Anemometer";


#define ANEMOMETER_SPEED_SAMPLES   5 // moving average over 5 seconds


// class
class AnemometerModule:  public DroneModule {
protected:

  float _speedSamples[ANEMOMETER_SPEED_SAMPLES];
  uint8_t _speedSample;
  unsigned long _lastSpeedSampleTime;
public:

  AnemometerModule(uint8_t id, DroneSystem* ds);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  static void ISR();

  void setup();
  void loop();


};

#endif
