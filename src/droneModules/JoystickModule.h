/*

Manages a JOYSTICK I2c temp, humidty and pressure sensor

*/
#ifndef JOYSTICK_MODULE_H
#define JOYSTICK_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"

#include "I2CBaseModule.h"


#define JOYSTICK_I2C_ADDRESS         8

#define JOYSTICK_PARAM_X             (I2CBASE_SUBCLASS_PARAM_START+0)  //10
#define JOYSTICK_PARAM_Y             (I2CBASE_SUBCLASS_PARAM_START+1)
#define JOYSTICK_PARAM_Z             (I2CBASE_SUBCLASS_PARAM_START+2)
#define JOYSTICK_PARAM_BUTTON        (I2CBASE_SUBCLASS_PARAM_START+3)
#define JOYSTICK_PARAM_INVERT        (I2CBASE_SUBCLASS_PARAM_START+4)

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

  JoystickModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void doReset();

  void setup();

  void loop();
};

#endif
