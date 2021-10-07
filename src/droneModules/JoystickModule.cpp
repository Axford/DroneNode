#include "JoystickModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"


JoystickModule::JoystickModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(JOYSTICK_STR_JOYSTICK));
   _pins[0] = 0;

   // pubs
   initParams(JOYSTICK_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[JOYSTICK_PARAM_X_E];
   param->param = JOYSTICK_PARAM_X;
   setParamName(FPSTR(DRONE_STR_XAXIS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[JOYSTICK_PARAM_Y_E];
   param->param = JOYSTICK_PARAM_Y;
   setParamName(FPSTR(DRONE_STR_YAXIS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

}


void JoystickModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

  DroneModule::parsePins(obj, _pins, (uint8_t)sizeof(_pins));
}


void JoystickModule::setup() {
  DroneModule::setup();

  if (_pins[0] > 0) {
    for (uint8_t i=0; i<JOYSTICK_AXES; i++) {
      pinMode(_pins[i], INPUT);
    }

  } else {
    Log.errorln(F("Undefined pin %d"), _pins[0]);
    disable();
  }
}


void JoystickModule::loop() {
  DroneModule::loop();

  // get sensor values
  _params[JOYSTICK_PARAM_X_E].data.f[0] = 2.0 * (analogRead(_pins[0]) / 4096.0) - 1;
  _params[JOYSTICK_PARAM_Y_E].data.f[0] = 2.0 * (analogRead(_pins[1]) / 4096.0) - 1;

  // publish param entries
  publishParamEntries();
}
