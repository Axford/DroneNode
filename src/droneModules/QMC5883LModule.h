/*

@type          QMC5883L
@inherits      I2CCompass
@description   Manages a QMC5883L I2C Compass

@config >>>
[QMC5883L= 5]
  name= "Compass"
  interval= 50
  bus= 0
  calibX= -6,0,6
  calibY= -6,0,6
  calibZ=-6,0,6
  trim= 180
  location= -1.8, 52, 100
  mode=1
  centre = -3.188, 4.167, -0.1043
  $location = @>GPS.location
  $roll = @>MPU6050.roll
  $pitch = @>MPU6050.pitch
  publish =heading, vector, calibX, calibY, calibZ
  publish = trim, mode, roll, pitch, raw
<<<

*/
#ifndef QMC5883L_MODULE_H
#define QMC5883L_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"
#include "I2CCompassModule.h"

#include "I2Cdev.h"
#include "QMC5883LCompass.h"

#define QMC5883L_I2C_ADDRESS  0x0D

// strings
static const char QMC5883L_STR_QMC5883L[] PROGMEM = "QMC5883L";


// class
class QMC5883LModule:  public I2CCompassModule {
protected:
  QMC5883LCompass *_sensor;

public:
  QMC5883LModule(uint8_t id, DroneSystem* ds);

  boolean initSensor();

  void getSensorValues();
};

#endif
