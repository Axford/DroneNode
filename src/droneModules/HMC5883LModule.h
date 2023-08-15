/*

@type          HMC5883L
@inherits      I2CCompass
@description   Manages a HMC5883L I2C Compass

@config >>>
[HMC5883L= 8]
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
#ifndef HMC5883L_MODULE_H
#define HMC5883L_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"
#include "I2CCompassModule.h"

#include "I2Cdev.h"
#include "HMC5883L.h"

#define HMC5883L_I2C_ADDRESS  0x1E  // write address, read address is +1

// strings
static const char HMC5883L_STR_HMC5883L[] PROGMEM = "HMC5883L";


// class
class HMC5883LModule:  public I2CCompassModule {
protected:
  
  HMC5883L *_sensor;
public:

  HMC5883LModule(uint8_t id, DroneSystem* ds);
  
  boolean initSensor();

  void getSensorValues();
};

#endif
