/*

@type          BME280
@inherits      I2CBase
@description   Manages a BME280 I2c temp, humidty and pressure sensor

@config >>>
[ BME280 = 14 ]
  name = BME280
  bus = 4
  publish = temperature, humidity, pressure, altitude
<<<

*/
#ifndef BME280_MODULE_H
#define BME280_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/*
@I2CAddress        0x76
*/ 
#define BME280_I2C_ADDRESS  0x76

// @pub 10;f;1;temperature;Temperature in degrees C
#define BME280_PARAM_TEMPERATURE   (I2CBASE_SUBCLASS_PARAM_START+0) // 10
// @pub 11;f;1;humidity;Humidity
#define BME280_PARAM_HUMIDITY      (I2CBASE_SUBCLASS_PARAM_START+1) // 11
// @pub 12;f;1;pressure;Pressure
#define BME280_PARAM_PRESSURE      (I2CBASE_SUBCLASS_PARAM_START+2) // 12
// @pub 13;f;1;altitude;Altitude in meters
#define BME280_PARAM_ALTITUDE      (I2CBASE_SUBCLASS_PARAM_START+3) // 13

#define BME280_PARAM_TEMPERATURE_E   (I2CBASE_PARAM_ENTRIES+0)
#define BME280_PARAM_HUMIDITY_E      (I2CBASE_PARAM_ENTRIES+1)
#define BME280_PARAM_PRESSURE_E      (I2CBASE_PARAM_ENTRIES+2)
#define BME280_PARAM_ALTITUDE_E      (I2CBASE_PARAM_ENTRIES+3)

#define BME280_PARAM_ENTRIES    (I2CBASE_PARAM_ENTRIES + 4)


// strings
static const char BME280_STR_BME280[] PROGMEM = "BME280";

// class
class BME280Module:  public I2CBaseModule {
protected:
  Adafruit_BME280 _sensor;
public:

  BME280Module(uint8_t id, DroneSystem* ds);

  void doReset();

  virtual void loop();
};

#endif
