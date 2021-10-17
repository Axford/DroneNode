/*

Manages a JOYSTICK I2c temp, humidty and pressure sensor

*/
#ifndef JOYSTICK_MODULE_H
#define JOYSTICK_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"

#include "I2CBaseModule.h"


#define JOYSTICK_I2C_ADDRESS         8

#define JOYSTICK_PARAM_X             8
#define JOYSTICK_PARAM_Y             9
#define JOYSTICK_PARAM_Z             10
#define JOYSTICK_PARAM_BUTTON        11

#define JOYSTICK_PARAM_X_E           0
#define JOYSTICK_PARAM_Y_E           1
#define JOYSTICK_PARAM_Z_E           2
#define JOYSTICK_PARAM_BUTTON_E      3

#define JOYSTICK_PARAM_ENTRIES       4

#define JOYSTICK_AXES                4


// strings
static const char JOYSTICK_STR_JOYSTICK[] PROGMEM = "Joystick";

// class
class JoystickModule:  public I2CBaseModule {
protected:
  boolean _invert[JOYSTICK_AXES];
public:

  JoystickModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);

  void doReset();

  void loadConfiguration(JsonObject &obj);

  void loop();
};

#endif
