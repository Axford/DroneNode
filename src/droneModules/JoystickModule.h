/*

@type          INA219
@inherits      I2CBase
@category      Input
@description   Manages I2C joystick using DroneJoystick firmware

@config >>>
[Joystick= 4]
  name= "LeftJoy"
  bus= 1
  invert=0
  publish = x,y,z,button,invert
<<<

*/
#ifndef JOYSTICK_MODULE_H
#define JOYSTICK_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"

#include "I2CBaseModule.h"

/*
@I2CAddress        0x08
@default addr = 8
*/ 
#define JOYSTICK_I2C_ADDRESS         8

// @pub 10;f;1;r;xAxis;X axis value
#define JOYSTICK_PARAM_X             (I2CBASE_SUBCLASS_PARAM_START+0)  //10

// @pub 11;f;1;r;yAxis;Y axis value
#define JOYSTICK_PARAM_Y             (I2CBASE_SUBCLASS_PARAM_START+1)

// @pub 12;f;1;r;zAxis;Z axis value
#define JOYSTICK_PARAM_Z             (I2CBASE_SUBCLASS_PARAM_START+2)

// @pub 13;f;1;r;button;Button value
#define JOYSTICK_PARAM_BUTTON        (I2CBASE_SUBCLASS_PARAM_START+3)

// @pub 14;u8;4;w;invert;Invert flags for each axis [x,y,z,button]
#define JOYSTICK_PARAM_INVERT        (I2CBASE_SUBCLASS_PARAM_START+4)  // 14

#define JOYSTICK_PARAM_X_E           (I2CBASE_PARAM_ENTRIES+0)
#define JOYSTICK_PARAM_Y_E           (I2CBASE_PARAM_ENTRIES+1)
#define JOYSTICK_PARAM_Z_E           (I2CBASE_PARAM_ENTRIES+2)
#define JOYSTICK_PARAM_BUTTON_E      (I2CBASE_PARAM_ENTRIES+3)
#define JOYSTICK_PARAM_INVERT_E      (I2CBASE_PARAM_ENTRIES+4)

#define JOYSTICK_PARAM_ENTRIES       (I2CBASE_PARAM_ENTRIES + 5)

#define JOYSTICK_AXES                4


// strings
static const char JOYSTICK_STR_JOYSTICK[] PROGMEM = "Joystick";

// class
class JoystickModule:  public I2CBaseModule {
protected:
  //boolean _invert[JOYSTICK_AXES];
public:

  JoystickModule(uint8_t id, DroneSystem* ds);

  void doReset();

  void setup();
  void loop();
};

#endif
