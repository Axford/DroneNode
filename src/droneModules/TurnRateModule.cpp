#include "TurnRateModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"


TurnRateModule::TurnRateModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(TURN_RATE_STR_TURN_RATE));

   // mgmt
   _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, sizeof(TURN_RATE_STR_TURN_RATE));
   strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, TURN_RATE_STR_TURN_RATE, sizeof(TURN_RATE_STR_TURN_RATE));


   // subs
   initSubs(TURN_RATE_SUBS);

   _subs[TURN_RATE_SUB_TARGET_E].addrParam = TURN_RATE_SUB_TARGET_ADDR;
   _subs[TURN_RATE_SUB_TARGET_E].param.param = TURN_RATE_SUB_TARGET;
   _subs[TURN_RATE_SUB_TARGET_E].param.name = FPSTR(DRONE_STR_TARGET);
   _subs[TURN_RATE_SUB_TARGET_E].param.nameLen = sizeof(DRONE_STR_TARGET);

   _subs[TURN_RATE_SUB_HEADING_E].addrParam = TURN_RATE_SUB_HEADING_ADDR;
   _subs[TURN_RATE_SUB_HEADING_E].param.param = TURN_RATE_SUB_HEADING;
   _subs[TURN_RATE_SUB_HEADING_E].param.name = FPSTR(DRONE_STR_HEADING);
   _subs[TURN_RATE_SUB_HEADING_E].param.nameLen = sizeof(DRONE_STR_HEADING);

   _subs[TURN_RATE_SUB_PID_E].addrParam = TURN_RATE_SUB_PID_ADDR;
   _subs[TURN_RATE_SUB_PID_E].param.param = TURN_RATE_SUB_PID;
   _subs[TURN_RATE_SUB_PID_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _subs[TURN_RATE_SUB_PID_E].param.name = FPSTR(DRONE_STR_PID);
   _subs[TURN_RATE_SUB_PID_E].param.nameLen = sizeof(DRONE_STR_PID);


   // outputs
   initParams(TURN_RATE_PARAM_ENTRIES);

   _params[TURN_RATE_PARAM_TURN_RATE_E].param = TURN_RATE_PARAM_TURN_RATE;
   _params[TURN_RATE_PARAM_TURN_RATE_E].name = FPSTR(DRONE_STR_TURN_RATE);
   _params[TURN_RATE_PARAM_TURN_RATE_E].nameLen = sizeof(DRONE_STR_TURN_RATE);
   _params[TURN_RATE_PARAM_TURN_RATE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
}


float TurnRateModule::getRotationDistance(float origin, float target){
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


void TurnRateModule::update() {
  // calc and publish new speeds

  // check we've received valid heading and target
  if (!_subs[TURN_RATE_SUB_HEADING_E].received ||
      !_subs[TURN_RATE_SUB_TARGET_E].received) return;

  // local shortcuts
  float h = _subs[TURN_RATE_SUB_HEADING_E].param.data.f[0];
  float t = _subs[TURN_RATE_SUB_TARGET_E].param.data.f[0];

  // calc shortest signed distance
  // positive values indicate a clockwise turn
  float err = getRotationDistance( h, t );

  // limit to 90 for ease
  if (err > 90) err = 90;
  if (err < -90) err = -90;

  _params[TURN_RATE_PARAM_TURN_RATE_E].data.f[0] = err * _subs[TURN_RATE_SUB_PID_E].param.data.f[0];

  publishParamEntry(&_params[TURN_RATE_PARAM_TURN_RATE_E]);
}
