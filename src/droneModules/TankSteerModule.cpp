#include "TankSteerModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"


TankSteerModule::TankSteerModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   // set type
   setTypeName(FPSTR(TANK_STEER_STR_TANK_STEER));

   // subs
   initSubs(TANK_STEER_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[TANK_STEER_SUB_TURN_RATE_E];
   sub->addrParam = TANK_STEER_SUB_TURN_RATE_ADDR;
   sub->param.param = TANK_STEER_SUB_TURN_RATE;
   setParamName(FPSTR(DRONE_STR_TURN_RATE), &sub->param);

   sub = &_subs[TANK_STEER_SUB_SPEED_E];
   sub->addrParam = TANK_STEER_SUB_SPEED_ADDR;
   sub->param.param = TANK_STEER_SUB_SPEED;
   setParamName(FPSTR(DRONE_STR_SPEED), &sub->param);

   // pubs
   initParams(TANK_STEER_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[TANK_STEER_PARAM_LEFT_E];
   param->param = TANK_STEER_PARAM_LEFT;
   setParamName(FPSTR(DRONE_STR_LEFT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[TANK_STEER_PARAM_RIGHT_E];
   param->param = TANK_STEER_PARAM_RIGHT;
   setParamName(FPSTR(DRONE_STR_RIGHT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   update();  // set defaults
}


float shortestSignedDistanceBetweenCircularValues(float origin, float target){
  float signedDiff = 0.0;
  float raw_diff = origin > target ? origin - target : target - origin;
  float mod_diff = fmod(raw_diff, 360); //equates rollover values. E.g 0 == 360 degrees in circle

  if(mod_diff > (360/2) ){
    //There is a shorter path in opposite direction
    signedDiff = (360 - mod_diff);
    if(target>origin) signedDiff = signedDiff * -1;
  } else {
    signedDiff = mod_diff;
    if(origin>target) signedDiff = signedDiff * -1;
  }

  return signedDiff;
}


void TankSteerModule::update() {
  // calc and publish new speeds

  // local shortcuts
  float x = _subs[TANK_STEER_SUB_TURN_RATE_E].param.data.f[0];

  // set target speed based on distance to go
  float y = _subs[TANK_STEER_SUB_SPEED_E].param.data.f[0];

  x = -x;
  float v = (1- abs(x)) * y + y;
  float w = (1-abs(y)) * x + x;

  float right = (v + w)/2;
  float left = (v-w)/2;

  _params[TANK_STEER_PARAM_LEFT_E].data.f[0] = left;
  _params[TANK_STEER_PARAM_RIGHT_E].data.f[0] = right;

  publishParamEntries();
}
