#include "NunchuckJoystickModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

NunchuckJoystick::NunchuckJoystick(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  I2CBaseModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(NunJOYSTICK_STR_NunJOYSTICK));
   //_pins[0] = 0;

   // pubs
   initParams(NunJOYSTICK_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = NunJOYSTICK_I2C_ADDRESS;

   DRONE_PARAM_ENTRY *param;

   param = &_params[NunJOYSTICK_PARAM_X_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NunJOYSTICK_PARAM_X);
   setParamName(FPSTR(STRING_XAXIS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[NunJOYSTICK_PARAM_Y_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NunJOYSTICK_PARAM_Y);
   setParamName(FPSTR(STRING_YAXIS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[NunJOYSTICK_PARAM_Z_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NunJOYSTICK_PARAM_Z);
   setParamName(FPSTR(STRING_ZAXIS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[NunJOYSTICK_PARAM_BUTTON_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NunJOYSTICK_PARAM_BUTTON);
   setParamName(FPSTR(STRING_BUTTON), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[NunJOYSTICK_PARAM_INVERT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NunJOYSTICK_PARAM_INVERT);
   setParamName(FPSTR(STRING_INVERT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 4);
   _params[NunJOYSTICK_PARAM_INVERT_E].data.uint8[0] = 0;
   _params[NunJOYSTICK_PARAM_INVERT_E].data.uint8[1] = 0;
   _params[NunJOYSTICK_PARAM_INVERT_E].data.uint8[2] = 0;
   _params[NunJOYSTICK_PARAM_INVERT_E].data.uint8[3] = 0;
}


DEM_NAMESPACE* NunchuckJoystick::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(NunJOYSTICK_STR_NunJOYSTICK,0,true);
}

void NunchuckJoystick::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {

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


void NunchuckJoystick::doReset() {
  I2CBaseModule::doReset();

  setError(0);
}


void NunchuckJoystick::setup() {
  I2CBaseModule::setup();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  //begin nunchuck
  nunchuck1.begin();
}


void NunchuckJoystick::loop() {
  I2CBaseModule::loop();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  //uint8_t bytes = Wire.requestFrom((uint16_t)_params[I2CBASE_PARAM_ADDR_E].data.uint8[0], (uint8_t)4, true);

  nunchuck1.readData();    // Read inputs and update maps

  	//Serial.println("-------------------------------------------");
	nunchuck1.readData();    // Read inputs and update maps

/*
  values[1]=map(getJoyX(),0,255,0,255);
	values[2]=map(getJoyY(),0,255,0,255);
	values[3]=map(getRollAngle(),0,1024,0,256);
	values[4]=map(getPitchAngle(),0,1024,0,256);
	values[5]=map(getAccelX(),0,1024,0,256);
	values[6]=map(getAccelY(),0,1024,0,256);

	values[7]=map(getAccelZ(),0,1024,0,256);
	values[8]=0;
	values[9]=0;
	values[10]=0;
	values[11]=getButtonZ()?255:0;
	values[12]=getButtonC()?255:0;
  */

  float v;

  // X
  v = nunchuck1.values[0];
  v = (v - 128) / 128.0f;   // remap to -1 .. 1
  if ((_params[NunJOYSTICK_PARAM_INVERT_E].data.uint8[NunJOYSTICK_PARAM_X_E] == 1)) v = -v;
  // if changed, then publish
  updateAndPublishParam(&_params[NunJOYSTICK_PARAM_X_E], (uint8_t*)&v, sizeof(v));

  // Y
  v = nunchuck1.values[1];
  v = (v - 128) / 128.0f;   // remap to -1 .. 1
  if ((_params[NunJOYSTICK_PARAM_INVERT_E].data.uint8[NunJOYSTICK_PARAM_Y_E] == 1)) v = -v;
  // if changed, then publish
  updateAndPublishParam(&_params[NunJOYSTICK_PARAM_Y_E], (uint8_t*)&v, sizeof(v));

  // Z Button
  v = nunchuck1.values[10];
  v = (v - 128) / 128.0f;   // remap to -1 .. 1
  if ((_params[NunJOYSTICK_PARAM_INVERT_E].data.uint8[NunJOYSTICK_PARAM_Z_E] == 1)) v = -v;
  // if changed, then publish
  updateAndPublishParam(&_params[NunJOYSTICK_PARAM_Z_E], (uint8_t*)&v, sizeof(v));

  // C  Button
  v = nunchuck1.values[11];
  v = (v - 128) / 128.0f;   // remap to -1 .. 1
  if ((_params[NunJOYSTICK_PARAM_INVERT_E].data.uint8[NunJOYSTICK_PARAM_BUTTON_E] == 1)) v = -v;
  // if changed, then publish
  updateAndPublishParam(&_params[NunJOYSTICK_PARAM_BUTTON_E], (uint8_t*)&v, sizeof(v));
    //Serial.println("");

//Serial.print("X: "); Serial.print(_params[NunJOYSTICK_PARAM_X_E].data.f[0]);
//Serial.print(", Y: "); Serial.print(_params[NunJOYSTICK_PARAM_Y_E].data.f[0]);
//Serial.print(", Z: "); Serial.print(_params[NunJOYSTICK_PARAM_Z_E].data.f[0]);
//Serial.print(", Button: "); Serial.println(_params[NunJOYSTICK_PARAM_BUTTON_E].data.f[0]);

}
