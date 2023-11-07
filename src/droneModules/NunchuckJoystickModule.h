/*

Manages a a Wii Nunchuk joystick Via ic2

*/
#ifndef NunJOYSTICK_MODULE_H
#define NunJOYSTICK_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"

#include "I2CBaseModule.h"
#include <WiiChuck.h>

#define NunJOYSTICK_I2C_ADDRESS         0x52

#define NunJOYSTICK_PARAM_X            (I2CBASE_SUBCLASS_PARAM_START+0)  //10
#define NunJOYSTICK_PARAM_Y             (I2CBASE_SUBCLASS_PARAM_START+1)
#define NunJOYSTICK_PARAM_Z             (I2CBASE_SUBCLASS_PARAM_START+2)
#define NunJOYSTICK_PARAM_BUTTON        (I2CBASE_SUBCLASS_PARAM_START+3)
#define NunJOYSTICK_PARAM_INVERT        (I2CBASE_SUBCLASS_PARAM_START+4)

#define NunJOYSTICK_PARAM_X_E           (I2CBASE_PARAM_ENTRIES+0)
#define NunJOYSTICK_PARAM_Y_E           (I2CBASE_PARAM_ENTRIES+1)
#define NunJOYSTICK_PARAM_Z_E           (I2CBASE_PARAM_ENTRIES+2)
#define NunJOYSTICK_PARAM_BUTTON_E      (I2CBASE_PARAM_ENTRIES+3)
#define NunJOYSTICK_PARAM_INVERT_E      (I2CBASE_PARAM_ENTRIES+4)

#define NunJOYSTICK_PARAM_ENTRIES       (I2CBASE_PARAM_ENTRIES + 5)

#define NunJOYSTICK_AXES                4


// strings
static const char NunJOYSTICK_STR_NunJOYSTICK[] PROGMEM = "Nunchuck";  // trying to keep strings short

// class
class NunchuckJoystick:  public I2CBaseModule {
protected:
  //boolean _invert[NunJOYSTICK_AXES];
  Accessory nunchuck1;

public:

  NunchuckJoystick(uint8_t id, DroneSystem* ds);

  void doReset();

  void setup();
  void loop();
};

#endif
