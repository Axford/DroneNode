#include "TurnRateModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

TurnRateModule::TurnRateModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(TURN_RATE_STR_TURN_RATE));

   _lastUpdate = 0;
   _iError = 0;
   _dError = 0;
   _lastError = 0;
   _lastHeading = 0;

   // mgmt
   //_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, sizeof(TURN_RATE_STR_TURN_RATE));
   //strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, TURN_RATE_STR_TURN_RATE, sizeof(TURN_RATE_STR_TURN_RATE));


   // subs
   initSubs(TURN_RATE_SUBS);

   _subs[TURN_RATE_SUB_TARGET_E].addrParam = TURN_RATE_SUB_TARGET_ADDR;
   _subs[TURN_RATE_SUB_TARGET_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TURN_RATE_SUB_TARGET);
   _subs[TURN_RATE_SUB_TARGET_E].param.name = FPSTR(STRING_TARGET);
   _subs[TURN_RATE_SUB_TARGET_E].param.nameLen = sizeof(STRING_TARGET);

   _subs[TURN_RATE_SUB_HEADING_E].addrParam = TURN_RATE_SUB_HEADING_ADDR;
   _subs[TURN_RATE_SUB_HEADING_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TURN_RATE_SUB_HEADING);
   _subs[TURN_RATE_SUB_HEADING_E].param.name = FPSTR(STRING_HEADING);
   _subs[TURN_RATE_SUB_HEADING_E].param.nameLen = sizeof(STRING_HEADING);

   _subs[TURN_RATE_SUB_PID_E].addrParam = TURN_RATE_SUB_PID_ADDR;
   _subs[TURN_RATE_SUB_PID_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TURN_RATE_SUB_PID);
   _subs[TURN_RATE_SUB_PID_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _subs[TURN_RATE_SUB_PID_E].param.name = FPSTR(STRING_PID);
   _subs[TURN_RATE_SUB_PID_E].param.nameLen = sizeof(STRING_PID);


   // pubs
   initParams(TURN_RATE_PARAM_ENTRIES);

   _params[TURN_RATE_PARAM_TURN_RATE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, TURN_RATE_PARAM_TURN_RATE);
   _params[TURN_RATE_PARAM_TURN_RATE_E].name = FPSTR(STRING_TURN_RATE);
   _params[TURN_RATE_PARAM_TURN_RATE_E].nameLen = sizeof(STRING_TURN_RATE);
   _params[TURN_RATE_PARAM_TURN_RATE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[TURN_RATE_PARAM_THRESHOLD_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TURN_RATE_PARAM_THRESHOLD);
   _params[TURN_RATE_PARAM_THRESHOLD_E].name = FPSTR(STRING_THRESHOLD);
   _params[TURN_RATE_PARAM_THRESHOLD_E].nameLen = sizeof(STRING_THRESHOLD);
   _params[TURN_RATE_PARAM_THRESHOLD_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[TURN_RATE_PARAM_THRESHOLD_E].data.f[0] = 20;

   _params[TURN_RATE_PARAM_TIMEOUT_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TURN_RATE_PARAM_TIMEOUT);
   _params[TURN_RATE_PARAM_TIMEOUT_E].name = FPSTR(STRING_TIMEOUT);
   _params[TURN_RATE_PARAM_TIMEOUT_E].nameLen = sizeof(STRING_TIMEOUT);
   _params[TURN_RATE_PARAM_TIMEOUT_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[TURN_RATE_PARAM_TIMEOUT_E].data.f[0] = 10;

   _params[TURN_RATE_PARAM_MODE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, TURN_RATE_PARAM_MODE);
   _params[TURN_RATE_PARAM_MODE_E].name = FPSTR(STRING_MODE);
   _params[TURN_RATE_PARAM_MODE_E].nameLen = sizeof(STRING_MODE);
   _params[TURN_RATE_PARAM_MODE_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[TURN_RATE_PARAM_MODE_E].data.uint8[0] = 0;

   _gybeTimerStart = TURN_RATE_MODE_NORMAL;
   _positiveError = false;
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


void TurnRateModule::loop() {
  if (!_setupDone) return;

  unsigned long updateTime = millis();
  float dt = (updateTime - _lastUpdate) / 1000.0;

  // don't bother updating if dt too small
  if (dt < 0.05) return;

  _lastUpdate = updateTime;

  // calc and publish new speeds

  // check we've received valid heading and target
  // this is actually very annoying, as it prevents manual control until a subscriber input is heard
  /*
  if (!_subs[TURN_RATE_SUB_HEADING_E].received ||
      !_subs[TURN_RATE_SUB_TARGET_E].received) return;
  */

  // local shortcuts
  float h = _subs[TURN_RATE_SUB_HEADING_E].param.data.f[0];
  float t = _subs[TURN_RATE_SUB_TARGET_E].param.data.f[0];

  // check to see if heading has dramatically changed
  if (fabs(getRotationDistance(_lastHeading, t)) > 45) {
    // reset d and i errors
    _iError = 0;
    _dError = 0;
  }

  // calc shortest signed distance
  // positive values indicate a clockwise turn
  float err = getRotationDistance( h, t );
  boolean positiveError = err > 0;

  uint8_t newMode = _params[TURN_RATE_PARAM_MODE_E].data.uint8[0];

  // check for potential gybe condition
  if (fabs(err) > _params[TURN_RATE_PARAM_THRESHOLD_E].data.f[0]) {
    if (newMode == TURN_RATE_MODE_NORMAL) {
      // if in normal mode, then change to potential gybe and start the timer
      newMode = TURN_RATE_MODE_POTENTIAL_GYBE;
      _gybeTimerStart = updateTime;
      _positiveError = err > 0;
    }

    if (positiveError != _positiveError) {
      // reset to normal mode
      //newMode = TURN_RATE_MODE_NORMAL;
      // dont reset to normal until error falls below threshold... i.e. gybe complete
    } else {
      if ( (updateTime - _gybeTimerStart)/1000.0 > _params[TURN_RATE_PARAM_TIMEOUT_E].data.f[0]) {
        // if in potential gybe and timer has exceed the timeout... switch to gybe mode
        newMode = TURN_RATE_MODE_GYBE;
      }
    }
  } else {
    // reset to normal mode
    newMode = TURN_RATE_MODE_NORMAL;
  }

  // if gybe mode then invert err
  if (newMode == TURN_RATE_MODE_GYBE) {
    // if _positiveError then when we first entered gybe we needed to turn clockwise to reach the targetHeading
    if ((err > 0 && _positiveError) || (err < 0 && !_positiveError)) {
      err = -err;
    }
  }

  // limit to 90 for ease
  if (err > 90) err = 90;
  if (err < -90) err = -90;

  // update I and D terms
  _iError += err * dt;
  _dError = (err - _lastError) / dt;

  // clamp i error
  if (_subs[TURN_RATE_SUB_PID_E].param.data.f[1] > 0) {
    if (fabs(_iError) > 100 / _subs[TURN_RATE_SUB_PID_E].param.data.f[1]) {
      _iError = (_iError > 0 ? 100 : -100) / _subs[TURN_RATE_SUB_PID_E].param.data.f[1];
    }
  }

  // apply PID cooefficients
  float tr =
    err * _subs[TURN_RATE_SUB_PID_E].param.data.f[0] +
    _iError * _subs[TURN_RATE_SUB_PID_E].param.data.f[1] +
    _dError * _subs[TURN_RATE_SUB_PID_E].param.data.f[2];

  _lastError = err;

  // apply limits
  if (tr > 10) tr = 10;
  if (tr < -10) tr = -10;

  updateAndPublishParam(&_params[TURN_RATE_PARAM_TURN_RATE_E], (uint8_t*)&tr, sizeof(tr));

  updateAndPublishParam(&_params[TURN_RATE_PARAM_MODE_E], (uint8_t*)&newMode, sizeof(newMode));

  _lastHeading = h;
}
