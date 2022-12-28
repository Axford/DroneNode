/*

Manages a MPU6050 I2c gyro/accel sensor

@config >>>
MPU6050.new 11
  name "MPU6050"
  status 0
  interval 1000
  .publish "gyro"
  .publish "accel"
.done
<<<

*/
#ifndef MPU6050_MODULE_H
#define MPU6050_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#define MPU6050_I2C_ADDRESS  0x68

#define MPU6050_PARAM_ACCEL     (I2CBASE_SUBCLASS_PARAM_START+0) // 10
#define MPU6050_PARAM_ACCEL_E   (I2CBASE_PARAM_ENTRIES+0)

#define MPU6050_PARAM_GYRO      (I2CBASE_SUBCLASS_PARAM_START+1) // 11
#define MPU6050_PARAM_GYRO_E    (I2CBASE_PARAM_ENTRIES+1)

#define MPU6050_PARAM_TEMPERATURE      (I2CBASE_SUBCLASS_PARAM_START+2) // 12
#define MPU6050_PARAM_TEMPERATURE_E    (I2CBASE_PARAM_ENTRIES+2)

#define MPU6050_PARAM_ENTRIES    (I2CBASE_PARAM_ENTRIES + 3)

// strings
static const char MPU6050_STR_MPU6050[] PROGMEM = "MPU6050";

// class
class MPU6050Module:  public I2CBaseModule {
protected:
  Adafruit_MPU6050 *_sensor;
public:

  MPU6050Module(uint8_t id, DroneSystem* ds);
  ~MPU6050Module();

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void doReset();

  void setup();
  void loop();
};

#endif
