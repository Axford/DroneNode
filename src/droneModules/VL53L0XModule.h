/*

@type          CVL53L0X
@inherits      I2CBase
@description   Manages a VL53L0X laser distance sensor

@config >>>
[VL53L0X = 8]
  name= "Laser"
  bus= 3
  publish = distance
<<<

*/
#ifndef VL53L0X_MODULE_H
#define VL53L0X_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "I2Cdev.h"

#include <VL53L0X.h>

/*
@I2CAddress        0x29
*/ 
#define VL53L0X_I2C_ADDRESS  0x29  // default address

// pubs
// pubs of form: <param address>;<type>;<number of values>;<name>;<description>


// @pub 10;f;1;distance;Distance in mm
#define VL53L0X_PARAM_DISTANCE         (I2CBASE_SUBCLASS_PARAM_START+0)  //10


#define VL53L0X_PARAM_DISTANCE_E         (I2CBASE_PARAM_ENTRIES+0)

#define VL53L0X_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 1)

// subs
// subs of form: <param address>;<addr param address>;<type>;<number of values>;<name>;description

#define VL53L0X_SUBS                    0

// strings
static const char VL53L0X_STR_VL53L0X[] PROGMEM = "VL53L0X";


// class
class VL53L0XModule:  public I2CBaseModule {
protected:
  VL53L0X *_sensor;
public:

  VL53L0XModule(uint8_t id, DroneSystem* ds);

  void doReset();

  void setup();
  void loop();
};

#endif
