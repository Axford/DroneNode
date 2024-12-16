#include "TankSteerModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../strings.h"
#include "../navMath.h"

// @type TankSteer

TankSteerModule::TankSteerModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
{
  // set type
  setTypeName(FPSTR(TANK_STEER_STR_TANK_STEER));

  _lastUpdate = 0;
  _iError = 0;
  _dError = 0;
  _lastError = 0;
  _lastHeading = 0;
  _lastMode = TANK_STEER_MODE_MANUAL;

  // subs
  initSubs(TANK_STEER_SUBS);

  DRONE_PARAM_SUB *sub;

  sub = &_subs[TANK_STEER_SUB_TARGET_E];
  sub->addrParam = TANK_STEER_SUB_TARGET_ADDR;
  sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TANK_STEER_SUB_TARGET);
  setParamName(FPSTR(STRING_TARGET), &sub->param);
  sub->param.data.f[0] = 0;
  sub->param.data.f[1] = 0;

  sub = &_subs[TANK_STEER_SUB_HEADING_E];
  sub->addrParam = TANK_STEER_SUB_HEADING_ADDR;
  sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TANK_STEER_SUB_HEADING);
  setParamName(FPSTR(STRING_HEADING), &sub->param);

  sub = &_subs[TANK_STEER_SUB_DISTANCE_E];
  sub->addrParam = TANK_STEER_SUB_DISTANCE_ADDR;
  sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TANK_STEER_SUB_DISTANCE);
  setParamName(FPSTR(STRING_DISTANCE), &sub->param);

  _subs[TANK_STEER_SUB_TARGET_E].enabled = false;
  _subs[TANK_STEER_SUB_DISTANCE_E].enabled = false;


  // pubs
  initParams(TANK_STEER_PARAM_ENTRIES);

  DRONE_PARAM_ENTRY *param;

  param = &_params[TANK_STEER_PARAM_LEFT_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, TANK_STEER_PARAM_LEFT);
  setParamName(FPSTR(STRING_LEFT), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

  param = &_params[TANK_STEER_PARAM_RIGHT_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, TANK_STEER_PARAM_RIGHT);
  setParamName(FPSTR(STRING_RIGHT), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

  param = &_params[TANK_STEER_PARAM_MODE_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, TANK_STEER_PARAM_MODE);
  setParamName(FPSTR(STRING_MODE), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
  param->data.uint8[0] = TANK_STEER_MODE_MANUAL;  // default to manual

  param = &_params[TANK_STEER_PARAM_TRIM_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TANK_STEER_PARAM_TRIM);
  setParamName(FPSTR(STRING_TRIM), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);

  param = &_params[TANK_STEER_PARAM_PID_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TANK_STEER_PARAM_PID);
  setParamName(FPSTR(STRING_PID), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
  // @default PID=0.01,0,0
  param->data.f[0] = 0.01;
  param->data.f[1] = 0;
  param->data.f[2] = 0;

  param = &_params[TANK_STEER_PARAM_LIMITS_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TANK_STEER_PARAM_LIMITS);
  setParamName(FPSTR(STRING_LIMITS), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
  param->data.f[0] = 0.2;
  param->data.f[1] = 0.7;

  param = &_params[TANK_STEER_PARAM_THRESHOLD_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TANK_STEER_PARAM_THRESHOLD);
  setParamName(FPSTR(STRING_THRESHOLD), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
  param->data.f[0] = 15;  // 15m

}



void TankSteerModule::loop() {
  DroneModule::loop();
  if (!_setupDone) return;

  if (_lastMode != _params[TANK_STEER_PARAM_MODE_E].data.uint8[0]) {
    if (_params[TANK_STEER_PARAM_MODE_E].data.uint8[0] == TANK_STEER_MODE_MANUAL) {
      // zero distance to stop motors when entering manual
      _subs[TANK_STEER_SUB_DISTANCE_E].param.data.f[0] = 0;

    }
    _lastMode = _params[TANK_STEER_PARAM_MODE_E].data.uint8[0];
  }

  if (_params[TANK_STEER_PARAM_MODE_E].data.uint8[0] == TANK_STEER_MODE_MANUAL) {
    _subs[TANK_STEER_SUB_TARGET_E].enabled = false;
    _subs[TANK_STEER_SUB_DISTANCE_E].enabled = false;
  } else {
    _subs[TANK_STEER_SUB_TARGET_E].enabled = true;
    _subs[TANK_STEER_SUB_DISTANCE_E].enabled = true;
  }
  

  // calc and publish new speeds
  float turnRate = 0;
  float speed = 0;

  // get distance to go
  float d = _subs[TANK_STEER_SUB_DISTANCE_E].param.data.f[0];
  float threshold = _params[TANK_STEER_PARAM_THRESHOLD_E].data.f[0];
  boolean considerReversing = d < threshold;
  boolean reversing = false;

  /*
    TURNRATE
  */
  unsigned long updateTime = millis();
  float dt = (updateTime - _lastUpdate) / 1000.0;

  // don't bother updating if dt too small
  if (dt < 0.05) return;

  // clamp dt, to avoid massive jumps
  if (dt > 1) dt = 1;

  _lastUpdate = updateTime;

  // calc and publish new speeds

  // check we've received valid heading and target
  // this is actually very annoying, as it prevents manual control until a subscriber input is heard
  /*
  if (!_subs[TANK_STEER_SUB_HEADING_E].received ||
      !_subs[TANK_STEER_SUB_TARGET_E].received) return;
  */

  // local shortcuts
  float h = _subs[TANK_STEER_SUB_HEADING_E].param.data.f[0];
  float t = _subs[TANK_STEER_SUB_TARGET_E].param.data.f[0];

  // check to see if heading has dramatically changed
  if (fabs(shortestSignedDistanceBetweenCircularValues(_lastHeading, t)) > 45) {
    // reset d and i errors
    _iError = 0;
    _dError = 0;
  }

  // calc shortest signed distance
  // positive values indicate a clockwise turn
  float err = shortestSignedDistanceBetweenCircularValues( h, t );

  if (considerReversing && fabs(err) > 90) {
    reversing = true;
    h += 180;
    h = fmod(h, 360);
    err = shortestSignedDistanceBetweenCircularValues( h, t );
  }

  boolean positiveError = err > 0;

  // limit to 90 for ease
  if (err > 90) err = 90;
  if (err < -90) err = -90;

  // update I and D terms
  _iError += err * dt;
  _dError = (err - _lastError) / dt;

  // clamp i error
  if (_params[TANK_STEER_PARAM_PID_E].data.f[1] > 0) {
    if (fabs(_iError) > 100 / _params[TANK_STEER_PARAM_PID_E].data.f[1]) {
      _iError = (_iError > 0 ? 100 : -100) / _params[TANK_STEER_PARAM_PID_E].data.f[1];
    }
  }

  // apply PID cooefficients
  turnRate =
    err * _params[TANK_STEER_PARAM_PID_E].data.f[0] +
    _iError * _params[TANK_STEER_PARAM_PID_E].data.f[1] +
    _dError * _params[TANK_STEER_PARAM_PID_E].data.f[2];

  _lastError = err;

  // apply limits
  if (turnRate > 10) turnRate = 10;
  if (turnRate < -10) turnRate = -10;

  _lastHeading = h;


  /*
    SPEED
  */

  // local shortcuts
  float smin = _params[TANK_STEER_PARAM_LIMITS_E].data.f[0];
  float smax = _params[TANK_STEER_PARAM_LIMITS_E].data.f[1];

  if (d > threshold) {
    speed = smax;
  } else {
    // lerp
    speed = (d/threshold) * (smax-smin) + smin;
  }

  // check for near zero distance.... and disable motors
  if (_subs[TANK_STEER_SUB_DISTANCE_E].param.data.f[0] < 1) {
    speed = 0;
    turnRate = 0;
    // also clamp iError to zero so it doesn't accumulate whilst we're stopped
    _iError = 0;
  }

  // local shortcuts
  float x = turnRate;
  // limit turnRate
  if (x > 1) x = 1;
  if (x < -1) x = -1;

  float y = reversing ? -speed : speed;
  // limit speed
  if (y > 1) y = 1;
  if (y < -1) y = -1;

  // use trim as offset to x value
  x += _params[TANK_STEER_PARAM_TRIM_E].data.f[0];

  x = -x;
  float v = (1- abs(x)) * y + y;
  float w = (1-abs(y)) * x + x;

  float right = (v + w)/2;
  float left = (v-w)/2;

  // limits
  if (right > 1) right = 1;
  if (right < -1) right = -1;
  if (left > 1) left = 1;
  if (left < -1) left = -1;

  //_params[TANK_STEER_PARAM_LEFT_E].data.f[0] = left;
  updateAndPublishParam(&_params[TANK_STEER_PARAM_LEFT_E], (uint8_t*)&left, sizeof(left));

  //_params[TANK_STEER_PARAM_RIGHT_E].data.f[0] = right;
  updateAndPublishParam(&_params[TANK_STEER_PARAM_RIGHT_E], (uint8_t*)&right, sizeof(right));

}
