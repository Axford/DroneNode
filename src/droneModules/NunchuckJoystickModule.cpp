#include "NunchuckJoystickModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

NunchuckJoystick::NunchuckJoystick(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem):
  I2CBaseModule ( id, dmm, dlm, dem )
 {
   setTypeName(FPSTR(NunJOYSTICK_STR_NunJOYSTICK));
   //_pins[0] = 0;
   _addr = NunJOYSTICK_I2C_ADDRESS;

   for (uint8_t i=0; i<NunJOYSTICK_AXES; i++) {
     _invert[i] = false;
   }

   // pubs
   initParams(NunJOYSTICK_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[NunJOYSTICK_PARAM_X_E];
   param->param = NunJOYSTICK_PARAM_X;
   setParamName(FPSTR(STRING_XAXIS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[NunJOYSTICK_PARAM_Y_E];
   param->param = NunJOYSTICK_PARAM_Y;
   setParamName(FPSTR(STRING_YAXIS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[NunJOYSTICK_PARAM_Z_E];
   param->param = NunJOYSTICK_PARAM_Z;
   setParamName(FPSTR(STRING_ZAXIS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[NunJOYSTICK_PARAM_BUTTON_E];
   param->param = NunJOYSTICK_PARAM_BUTTON;
   setParamName(FPSTR(STRING_BUTTON), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

}

void NunchuckJoystick::doReset() {
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_bus);

  setError(0);
}



void NunchuckJoystick::loadConfiguration(JsonObject &obj) {

  I2CBaseModule::loadConfiguration(obj);



  // load inversion settings
  if (obj.containsKey(STRING_INVERT)) {
    Log.noticeln(STRING_INVERT);

    if (obj[STRING_INVERT].is<JsonArray>()) {
      JsonArray array = obj[STRING_INVERT].as<JsonArray>();

      uint8_t i = 0;
      for(JsonVariant v : array) {
        if (i<NunJOYSTICK_AXES) {
          _invert[i] = v | _invert[i];
        }
        i++;
      }
    }
  }
}

void NunchuckJoystick::setup() {
  I2CBaseModule::setup();

  //begin nunchuck
  nunchuck1.begin();
}


void NunchuckJoystick::loop() {
  I2CBaseModule::loop();

  DroneWire::selectChannel(_bus);

  //uint8_t bytes = Wire.requestFrom((uint16_t)_addr, (uint8_t)4, true);

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
  if (_invert[NunJOYSTICK_PARAM_X_E]) v = -v;
  // if changed, then publish
  updateAndPublishParam(&_params[NunJOYSTICK_PARAM_X_E], (uint8_t*)&v, sizeof(v));

  // Y
  v = nunchuck1.values[1];
  v = (v - 128) / 128.0f;   // remap to -1 .. 1
  if (_invert[NunJOYSTICK_PARAM_Y_E]) v = -v;
  // if changed, then publish
  updateAndPublishParam(&_params[NunJOYSTICK_PARAM_Y_E], (uint8_t*)&v, sizeof(v));

  // Z Button
  v = nunchuck1.values[10];
  v = (v - 128) / 128.0f;   // remap to -1 .. 1
  if (_invert[NunJOYSTICK_PARAM_Z_E]) v = -v;
  // if changed, then publish
  updateAndPublishParam(&_params[NunJOYSTICK_PARAM_Z_E], (uint8_t*)&v, sizeof(v));

  // C  Button
  v = nunchuck1.values[11];
  v = (v - 128) / 128.0f;   // remap to -1 .. 1
  if (_invert[NunJOYSTICK_PARAM_BUTTON_E]) v = -v;
  // if changed, then publish
  updateAndPublishParam(&_params[NunJOYSTICK_PARAM_BUTTON_E], (uint8_t*)&v, sizeof(v));
    //Serial.println("");

//Serial.print("X: "); Serial.print(_params[NunJOYSTICK_PARAM_X_E].data.f[0]);
//Serial.print(", Y: "); Serial.print(_params[NunJOYSTICK_PARAM_Y_E].data.f[0]);
//Serial.print(", Z: "); Serial.print(_params[NunJOYSTICK_PARAM_Z_E].data.f[0]);
//Serial.print(", Button: "); Serial.println(_params[NunJOYSTICK_PARAM_BUTTON_E].data.f[0]);

}
