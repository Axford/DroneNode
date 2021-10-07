/*

Manages a JOYSTICK I2c temp, humidty and pressure sensor

*/
#ifndef JOYSTICK_MODULE_H
#define JOYSTICK_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"

#define JOYSTICK_I2C_ADDRESS  0x76

#define JOYSTICK_PARAM_X             8
#define JOYSTICK_PARAM_Y             9

#define JOYSTICK_PARAM_X_E           0
#define JOYSTICK_PARAM_Y_E           1

#define JOYSTICK_PARAM_ENTRIES       2

#define JOYSTICK_AXES                2

// strings
static const char JOYSTICK_STR_JOYSTICK[] PROGMEM = "Joystick";

// class
class JoystickModule:  public DroneModule {
protected:
  uint8_t _pins[JOYSTICK_AXES];
public:

  JoystickModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm);

  void loadConfiguration(JsonObject &obj);

  void setup();
  void loop();
};

#endif
