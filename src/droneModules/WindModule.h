/*

Manages a Wind speed and direction sensor

- Wind direction using I2C AS5600 sensor
- Wind speed from cup anemometor on an digital input with internal pullup

*/
#ifndef WIND_MODULE_H
#define WIND_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

//#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883_U.h>

#include <AS5600.h>

#define WIND_I2C_ADDRESS  0x36

// pubs
// local wind direction
#define WIND_PARAM_DIRECTION       (I2CBASE_SUBCLASS_PARAM_START+0)  //10
#define WIND_PARAM_SPEED           (I2CBASE_SUBCLASS_PARAM_START+1)  // 11
#define WIND_PARAM_PINS            (I2CBASE_SUBCLASS_PARAM_START+2)  // 12
// calibration for local wind direction
#define WIND_PARAM_CENTRE          (I2CBASE_SUBCLASS_PARAM_START+3)  // 13
// world wind direction
#define WIND_PARAM_WIND            (I2CBASE_SUBCLASS_PARAM_START+4)  // 14

#define WIND_PARAM_DIRECTION_E     (I2CBASE_PARAM_ENTRIES+0)
#define WIND_PARAM_SPEED_E         (I2CBASE_PARAM_ENTRIES+1)
#define WIND_PARAM_PINS_E          (I2CBASE_PARAM_ENTRIES+2)
#define WIND_PARAM_CENTRE_E        (I2CBASE_PARAM_ENTRIES+3)
#define WIND_PARAM_WIND_E          (I2CBASE_PARAM_ENTRIES+4)

#define WIND_PARAM_ENTRIES         (I2CBASE_PARAM_ENTRIES + 5)

// subs
// sub to compass heading to work out world wind direction
#define WIND_SUB_HEADING            (I2CBASE_SUBCLASS_PARAM_START+5)
#define WIND_SUB_HEADING_ADDR       (I2CBASE_SUBCLASS_PARAM_START+6)
#define WIND_SUB_HEADING_E          0

#define WIND_SUBS                   1

// strings
static const char WIND_STR_WIND[] PROGMEM = "Wind";

#define WIND_SAMPLES  5 // moving average over 5 seconds

// class
class WindModule:  public I2CBaseModule {
protected:
  AMS_5600 *_sensor;

  float _samples[WIND_SAMPLES];
  uint8_t _sample;
  unsigned long _lastSampleTime;
public:

  WindModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);
  ~WindModule();

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void doReset();

  static void ISR();

  void setup();
  void loop();


};

#endif
