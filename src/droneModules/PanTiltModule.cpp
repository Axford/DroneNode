#include "PanTiltModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"

PanTiltModule::PanTiltModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(PAN_TILT_STR_PAN_TILT));

   _lastUpdate = 0;
   _iError = 0;
   _dError = 0;
   _lastError = 0;
   _lastTarget = 0;

   // set default interval to 1000
   // @default interval = 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   // subs
   initSubs(PAN_TILT_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[PAN_TILT_SUB_TARGET_E];
   sub->addrParam = PAN_TILT_SUB_TARGET_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PAN_TILT_SUB_TARGET);
   setParamName(FPSTR(STRING_TARGET), &sub->param);

   sub = &_subs[PAN_TILT_SUB_HEADING_E];
   sub->addrParam = PAN_TILT_SUB_HEADING_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PAN_TILT_SUB_HEADING);
   setParamName(FPSTR(STRING_HEADING), &sub->param);


   // pubs
   initParams(PAN_TILT_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[PAN_TILT_PARAM_PID_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PAN_TILT_PARAM_PID);
   setParamName(FPSTR(STRING_PID), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[PAN_TILT_PARAM_PID_E].data.f[0] = 0.005;
   _params[PAN_TILT_PARAM_PID_E].data.f[1] = 0;
   _params[PAN_TILT_PARAM_PID_E].data.f[2] = 0.0001;

   param = &_params[PAN_TILT_PARAM_PAN_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PAN_TILT_PARAM_PAN);
   setParamName(FPSTR(STRING_PAN), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[PAN_TILT_PARAM_PAN_E].data.f[0] = 0;

   param = &_params[PAN_TILT_PARAM_LIMITS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PAN_TILT_PARAM_LIMITS);
   setParamName(FPSTR(STRING_LIMITS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   _params[PAN_TILT_PARAM_LIMITS_E].data.f[0] = -90;
   _params[PAN_TILT_PARAM_LIMITS_E].data.f[1] = 90;
}


float PanTiltModule::getRotationDistance(float origin, float target){
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



void PanTiltModule::update() {
  if (!_setupDone) return;

  unsigned long updateTime = millis();
  float dt = (updateTime - _lastUpdate) / 1000.0;

  // check we've received valid heading and target
  if (!_subs[PAN_TILT_SUB_HEADING_E].received ||
      !_subs[PAN_TILT_SUB_TARGET_E].received) return;

  // local shortcuts
  float h = _subs[PAN_TILT_SUB_HEADING_E].param.data.f[0];
  float t = _subs[PAN_TILT_SUB_TARGET_E].param.data.f[0];

  // calc shortest signed distance
  // positive values indicate a clockwise turn
  float panTarget = getRotationDistance( h, t );

  // limit range
  panTarget = constrain(panTarget, _params[PAN_TILT_PARAM_LIMITS_E].data.f[0], _params[PAN_TILT_PARAM_LIMITS_E].data.f[1]);

  float err = panTarget - _lastTarget;

  // update I and D terms
  _iError += err * dt;
  _dError = (err - _lastError) / dt;

  // apply PID cooefficients
  float delta =
    err * _subs[PAN_TILT_PARAM_PID_E].param.data.f[0] +
    _iError * _subs[PAN_TILT_PARAM_PID_E].param.data.f[1] +
    _dError * _subs[PAN_TILT_PARAM_PID_E].param.data.f[2];

  _lastError = err;

  // rate limit to 30 degrees/second
  //delta = constrain(delta, -30*dt, 30*dt);

  // update target
  //panTarget = _lastTarget + delta;

  // moving average
  panTarget = (_lastTarget * 7 + panTarget) / 8;

  _lastTarget = panTarget;

  // map to range -1 to 1
  float pan = mapF(panTarget, _params[PAN_TILT_PARAM_LIMITS_E].data.f[0], _params[PAN_TILT_PARAM_LIMITS_E].data.f[1], -1, 1);

  // limit range (again, just to be sure)
  pan = constrain(pan, -1, 1);

  updateAndPublishParam(&_params[PAN_TILT_PARAM_PAN_E], (uint8_t*)&panTarget, sizeof(panTarget));
}
