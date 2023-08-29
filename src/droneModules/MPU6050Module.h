/*

@type          MPU6050
@inherits      I2CBase
@description   Manages a MPU6050 I2c gyro/accel sensor

@config >>>
[MPU6050 = 14 ]
  name = "MPU6050"
  bus = 4
  interval = 50
  publish = accel, pitch, roll
  publish = raw, calibX, calibY, calibZ, mode
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

/// PUBS
// @pub 10;f;3;accel;Acceleration vector in G
#define MPU6050_PARAM_ACCEL         (I2CBASE_SUBCLASS_PARAM_START+0) // 10
// @pub 11;f;3;gyro;Gyro acceleration vector
#define MPU6050_PARAM_GYRO          (I2CBASE_SUBCLASS_PARAM_START+1) // 11
// @pub 12;f;1;temperature;Temperature in degrees C
#define MPU6050_PARAM_TEMPERATURE   (I2CBASE_SUBCLASS_PARAM_START+2) // 12
// @pub 13;f;1;pitch;Pitch angle in degrees, negative values are pitch down
#define MPU6050_PARAM_PITCH         (I2CBASE_SUBCLASS_PARAM_START+3)  // 13
// @pub 14;f;1;rol;Roll angle in degrees, negative values are roll right
#define MPU6050_PARAM_ROLL          (I2CBASE_SUBCLASS_PARAM_START+4) // 14

// @pub 15;f;4;raw;Raw acceleration vector, 4th value is std dev of rolling window
#define MPU6050_PARAM_RAW             (I2CBASE_SUBCLASS_PARAM_START+5)  // 15
// @pub 16;f;3;calibX;Min, center and max magnetic readings for the X axis
#define MPU6050_PARAM_CALIB_X         (I2CBASE_SUBCLASS_PARAM_START+6)  // 16
// @pub 17;f;3;calibY;Min, center and max magnetic readings for the Y axis
#define MPU6050_PARAM_CALIB_Y         (I2CBASE_SUBCLASS_PARAM_START+7)  // 17
// @pub 18;f;3;calibZ;Min, center and max magnetic readings for the Z axis
#define MPU6050_PARAM_CALIB_Z         (I2CBASE_SUBCLASS_PARAM_START+8)  // 18
// @pub 19;u8;1;mode;Mode: 0=online calibration, 1=fixed calibration, 2=reset calibration, 3=store calibration
#define MPU6050_PARAM_MODE            (I2CBASE_SUBCLASS_PARAM_START+9)  // 19

// @pub 20;f;3;calibG;Gyro calibration values (bias to zero)
#define MPU6050_PARAM_CALIB_G         (I2CBASE_SUBCLASS_PARAM_START+10)  // 20

#define MPU6050_PARAM_ACCEL_E         (I2CBASE_PARAM_ENTRIES+0)
#define MPU6050_PARAM_GYRO_E          (I2CBASE_PARAM_ENTRIES+1)
#define MPU6050_PARAM_TEMPERATURE_E   (I2CBASE_PARAM_ENTRIES+2)
#define MPU6050_PARAM_PITCH_E         (I2CBASE_PARAM_ENTRIES+3)
#define MPU6050_PARAM_ROLL_E          (I2CBASE_PARAM_ENTRIES+4)
#define MPU6050_PARAM_RAW_E           (I2CBASE_PARAM_ENTRIES+5)
#define MPU6050_PARAM_CALIB_X_E       (I2CBASE_PARAM_ENTRIES+6)
#define MPU6050_PARAM_CALIB_Y_E       (I2CBASE_PARAM_ENTRIES+7)
#define MPU6050_PARAM_CALIB_Z_E       (I2CBASE_PARAM_ENTRIES+8)
#define MPU6050_PARAM_MODE_E          (I2CBASE_PARAM_ENTRIES+9)
#define MPU6050_PARAM_CALIB_G_E       (I2CBASE_PARAM_ENTRIES+10)

#define MPU6050_PARAM_ENTRIES         (I2CBASE_PARAM_ENTRIES + 11)

// strings
static const char MPU6050_STR_MPU6050[] PROGMEM = "MPU6050";

#define MPU6050_MODE_ONLINE_CALIBRATION     0
#define MPU6050_MODE_FIXED_CALIBRATION      1
#define MPU6050_MODE_RESET_CALIBRATION      2
#define MPU6050_MODE_STORE_CALIBRATION      3


#define MPU6050_MOVING_AVERAGE_POINTS       20


// class
class MPU6050Module:  public I2CBaseModule {
protected:
  Adafruit_MPU6050 *_sensor;

  // moving average on raw vector values
  float _raw[3];  // raw values straight from the sensor
  float _rawG[3];  // raw gyro values
  float _lastRaw[3];
  float _rawAvg[3];
  float _lastMags[MPU6050_MOVING_AVERAGE_POINTS];
  uint8_t _lastMagIndex;
  boolean _lastMagFull;
  float _magAvg;
  float _magVariance;
  float _magDSquared;
  float _magStdDev;

  // min and max limits for raw magnetic values in all orientations
  float _minRaw[3];
  float _maxRaw[3];

  unsigned long _lastSampleMicros;
  unsigned long _lastPublishTime;
public:

  MPU6050Module(uint8_t id, DroneSystem* ds);
  ~MPU6050Module();
  
  void doReset();

  void updateCalibrationValuesFromRaw();

  void getSensorValues();

  void setup();
  void loop();
};

#endif
