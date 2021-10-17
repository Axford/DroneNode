#include "JoystickModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

JoystickModule::JoystickModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem):
  I2CBaseModule ( id, dmm, dlm, dem )
 {
   setTypeName(FPSTR(JOYSTICK_STR_JOYSTICK));
   //_pins[0] = 0;
   _addr = JOYSTICK_I2C_ADDRESS;

   for (uint8_t i=0; i<JOYSTICK_AXES; i++) {
     _invert[i] = false;
   }

   // pubs
   initParams(JOYSTICK_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[JOYSTICK_PARAM_X_E];
   param->param = JOYSTICK_PARAM_X;
   setParamName(FPSTR(STRING_XAXIS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[JOYSTICK_PARAM_Y_E];
   param->param = JOYSTICK_PARAM_Y;
   setParamName(FPSTR(STRING_YAXIS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[JOYSTICK_PARAM_Z_E];
   param->param = JOYSTICK_PARAM_Z;
   setParamName(FPSTR(STRING_ZAXIS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[JOYSTICK_PARAM_BUTTON_E];
   param->param = JOYSTICK_PARAM_BUTTON;
   setParamName(FPSTR(STRING_BUTTON), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

}

void JoystickModule::doReset() {
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_bus);

  setError(0);
}



void JoystickModule::loadConfiguration(JsonObject &obj) {
  I2CBaseModule::loadConfiguration(obj);

  // load inversion settings
  if (obj.containsKey(STRING_INVERT)) {
    Log.noticeln(STRING_INVERT);

    if (obj[STRING_INVERT].is<JsonArray>()) {
      JsonArray array = obj[STRING_INVERT].as<JsonArray>();

      uint8_t i = 0;
      for(JsonVariant v : array) {
        if (i<JOYSTICK_AXES) {
          _invert[i] = v | _invert[i];
        }
        i++;
      }
    }
  }
}


void JoystickModule::loop() {
  I2CBaseModule::loop();

  DroneWire::selectChannel(_bus);

  uint8_t bytes = Wire.requestFrom((uint16_t)_addr, (uint8_t)4, true);

  //Serial.print("!");

  uint8_t c;
  float v;

  if (bytes > 0) {
    for (uint8_t i=0; i<bytes; i++) {
      c = Wire.read();
      //Serial.print(c);
      //Serial.print(" ");
      if (i < JOYSTICK_AXES) {
        v = (c - 128) / 128.0f;
        if (_invert[i]) v = -v;

        // if changed, then publish
        if (_params[i].publish && v != _params[JOYSTICK_PARAM_X_E + i].data.f[0]) {
          _params[JOYSTICK_PARAM_X_E + i].data.f[0] = v;
          publishParamEntry(&_params[i]);
        }
      }
    }
    //Serial.println("");
  }
}
