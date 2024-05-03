#include "KiteControllerModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../strings.h"
#include "../navMath.h"

// @type KiteController

KiteControllerModule::KiteControllerModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
{
  // set type
  setTypeName(FPSTR(KITE_CONTROLLER_STR_KITE_CONTROLLER));

  _lastUpdate = 0;
  _lastMode = KITE_CONTROLLER_MODE_MANUAL;

  // subs
  initSubs(KITE_CONTROLLER_SUBS);

  DRONE_PARAM_SUB *sub;

  sub = &_subs[KITE_CONTROLLER_SUB_TURN_RATE_E];
  sub->addrParam = KITE_CONTROLLER_SUB_TURN_RATE_ADDR;
  sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, KITE_CONTROLLER_SUB_TURN_RATE);
  setParamName(FPSTR(STRING_TURN_RATE), &sub->param);
  sub->param.data.f[0] = 0;

  sub = &_subs[KITE_CONTROLLER_SUB_YAW_E];
  sub->addrParam = KITE_CONTROLLER_SUB_YAW_ADDR;
  sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, KITE_CONTROLLER_SUB_YAW);
  setParamName(FPSTR(STRING_YAW), &sub->param);

  sub = &_subs[KITE_CONTROLLER_SUB_PITCH_E];
  sub->addrParam = KITE_CONTROLLER_SUB_PITCH_ADDR;
  sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, KITE_CONTROLLER_SUB_PITCH);
  setParamName(FPSTR(STRING_PITCH), &sub->param);


  // pubs
  initParams(KITE_CONTROLLER_PARAM_ENTRIES);

  DRONE_PARAM_ENTRY *param;

  param = &_params[KITE_CONTROLLER_PARAM_LEFT_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, KITE_CONTROLLER_PARAM_LEFT);
  setParamName(FPSTR(STRING_LEFT), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

  param = &_params[KITE_CONTROLLER_PARAM_RIGHT_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, KITE_CONTROLLER_PARAM_RIGHT);
  setParamName(FPSTR(STRING_RIGHT), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

  param = &_params[KITE_CONTROLLER_PARAM_MODE_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, KITE_CONTROLLER_PARAM_MODE);
  setParamName(FPSTR(STRING_MODE), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
  param->data.uint8[0] = KITE_CONTROLLER_MODE_MANUAL;  // default to manual

  param = &_params[KITE_CONTROLLER_PARAM_TRIM_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, KITE_CONTROLLER_PARAM_TRIM);
  setParamName(FPSTR(STRING_TRIM), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);

  param = &_params[KITE_CONTROLLER_PARAM_LIMITS_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, KITE_CONTROLLER_PARAM_LIMITS);
  setParamName(FPSTR(STRING_LIMITS), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
  param->data.f[0] = -1;
  param->data.f[1] = 1;

}



void KiteControllerModule::loop() {
  DroneModule::loop();
  if (!_setupDone) return;

  
  // get inputs
  float tr = _subs[KITE_CONTROLLER_SUB_TURN_RATE_E].param.data.f[0];
  float yaw = _subs[KITE_CONTROLLER_SUB_YAW_E].param.data.f[0];
  float pitch = _subs[KITE_CONTROLLER_SUB_PITCH_E].param.data.f[0];

  // get params
  float trim = _params[KITE_CONTROLLER_PARAM_TRIM_E].data.f[0];
  float mmin = _params[KITE_CONTROLLER_PARAM_LIMITS_E].data.f[0];
  float mmax = _params[KITE_CONTROLLER_PARAM_LIMITS_E].data.f[1];


  float left = 0;
  float right = 0;


  // scale left/right using lerp to limits
  left = mmin + (mmax - mmin) * (left + 1) / 2;
  right = mmin + (mmax - mmin) * (right + 1) / 2;

  // apply trim
  left -= trim;
  right += trim;

  //_params[KITE_CONTROLLER_PARAM_LEFT_E].data.f[0] = left;
  updateAndPublishParam(&_params[KITE_CONTROLLER_PARAM_LEFT_E], (uint8_t*)&left, sizeof(left));

  //_params[KITE_CONTROLLER_PARAM_RIGHT_E].data.f[0] = right;
  updateAndPublishParam(&_params[KITE_CONTROLLER_PARAM_RIGHT_E], (uint8_t*)&right, sizeof(right));

}