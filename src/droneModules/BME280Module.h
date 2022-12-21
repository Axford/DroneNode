/*

Manages a BME280 I2c temp, humidty and pressure sensor

*/
#ifndef BME280_MODULE_H
#define BME280_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME280_I2C_ADDRESS  0x76

#define BME280_PARAM_TEMPERATURE   B00001000
#define BME280_PARAM_HUMIDITY      B00010000
#define BME280_PARAM_PRESSURE      B00100000
#define BME280_PARAM_ALTITUDE      B01000000

#define BME280_PARAM_TEMPERATURE_E   0
#define BME280_PARAM_HUMIDITY_E      1
#define BME280_PARAM_PRESSURE_E      2
#define BME280_PARAM_ALTITUDE_E      3

#define BME280_PARAM_ENTRIES    4


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
