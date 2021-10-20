#include "JoystickModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

JoystickModule::JoystickModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem):
  I2CBaseModule ( id, dmm, dlm, dem )
 {
   setTypeName(FPSTR(JOYSTICK_STR_JOYSTICK));

   // pubs
   initParams(JOYSTICK_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = JOYSTICK_I2C_ADDRESS;

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

   param = &_params[JOYSTICK_PARAM_INVERT_E];
   param->param = JOYSTICK_PARAM_INVERT;
   setParamName(FPSTR(STRING_INVERT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 4);
   _params[JOYSTICK_PARAM_INVERT_E].data.uint8[0] = 0;
   _params[JOYSTICK_PARAM_INVERT_E].data.uint8[1] = 0;
   _params[JOYSTICK_PARAM_INVERT_E].data.uint8[2] = 0;
   _params[JOYSTICK_PARAM_INVERT_E].data.uint8[3] = 0;

}


DEM_NAMESPACE* JoystickModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(JOYSTICK_STR_JOYSTICK,0,true);
}

void JoystickModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {

  I2CBaseModule::registerParams(ns, dem);

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_XAXIS, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_YAXIS, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_ZAXIS, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_BUTTON, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_INVERT, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
}


void JoystickModule::doReset() {
  I2CBaseModule::doReset();

  //DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  setError(0);
}


void JoystickModule::setup() {
  I2CBaseModule::setup();

}


void JoystickModule::loop() {
  I2CBaseModule::loop();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  uint8_t bytes = Wire.requestFrom((uint16_t)_params[I2CBASE_PARAM_ADDR_E].data.uint8[0], (uint8_t)4, true);

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
        if (_params[JOYSTICK_PARAM_INVERT_E].data.uint8[i] == 1) v = -v;

        // if changed, then publish
        if (_params[JOYSTICK_PARAM_X_E + i].publish && v != _params[JOYSTICK_PARAM_X_E + i].data.f[0]) {
          _params[JOYSTICK_PARAM_X_E + i].data.f[0] = v;
          publishParamEntry(&_params[JOYSTICK_PARAM_X_E + i]);
        }
      }
    }
    //Serial.println("");
  }
}
