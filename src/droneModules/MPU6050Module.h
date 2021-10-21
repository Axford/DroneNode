/*

Manages a MPU6050 I2c temp, humidty and pressure sensor

*/
#ifndef MPU6050_MODULE_H
#define MPU6050_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#define MPU6050_I2C_ADDRESS  0x68

#define MPU6050_PARAM_ACCEL     8
#define MPU6050_PARAM_ACCEL_E   0

#define MPU6050_PARAM_GYRO      9
#define MPU6050_PARAM_GYRO_E    1

#define MPU6050_PARAM_ENTRIES    2

// strings
static const char MPU6050_STR_MPU6050[] PROGMEM = "MPU6050";

// class
class MPU6050Module:  public I2CBaseModule {
protected:
  Adafruit_MPU6050 _sensor;
public:

  MPU6050Module(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  void doReset();

  virtual void loop();
};

#endif
