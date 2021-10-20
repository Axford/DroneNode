/*

Manages a a Wii Nunchuk joystick Via ic2

*/
#ifndef NunJOYSTICK_MODULE_H
#define NunJOYSTICK_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"

#include "I2CBaseModule.h"
#include <WiiChuck.h>

#define NunJOYSTICK_I2C_ADDRESS         52

#define NunJOYSTICK_PARAM_X             8
#define NunJOYSTICK_PARAM_Y             9
#define NunJOYSTICK_PARAM_Z             10
#define NunJOYSTICK_PARAM_BUTTON        11

#define NunJOYSTICK_PARAM_X_E           0
#define NunJOYSTICK_PARAM_Y_E           1
#define NunJOYSTICK_PARAM_Z_E           2
#define NunJOYSTICK_PARAM_BUTTON_E      3

#define NunJOYSTICK_PARAM_ENTRIES       4

#define NunJOYSTICK_AXES                4


// strings
static const char NunJOYSTICK_STR_NunJOYSTICK[] PROGMEM = "NunchuckJoystick";

// class
class NunchuckJoystick:  public I2CBaseModule {
protected:
  boolean _invert[NunJOYSTICK_AXES];
  Accessory nunchuck1;

public:

  NunchuckJoystick(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);

  void doReset();

  void setup();
  void loop();
};

#endif
