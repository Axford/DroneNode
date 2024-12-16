#include "KiteControllerModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../strings.h"
#include "../navMath.h"

#define SQR(x) ((x)*(x))

// @type KiteController

KiteControllerModule::KiteControllerModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
{
  // set type
  setTypeName(FPSTR(KITE_CONTROLLER_STR_KITE_CONTROLLER));

  _lastUpdate = 0;
  _lastMode = KITE_CONTROLLER_MODE_MANUAL;
  _roll = 0;
  _lastPos[0] = 0;
  _lastPos[1] = 0;
  _payout = 0;

  _waypoint = 0;
  _waypoints = IvanLinkedList::LinkedList<KITE_CONTROLLER_MODULE_WAYPOINT>();

  // prep waypoints - centred about origin, ie. in prep to be positioned for target pitch/yaw
  KITE_CONTROLLER_MODULE_WAYPOINT t;
  t.radius = 5;

  // use normalised dimensions, so they can be later scaled by target values
  t.yaw = 1;
  t.pitch = 1;
  _waypoints.add(t);

  t.yaw = -1;
  t.pitch = -1;
  _waypoints.add(t);

  t.yaw = -1;
  t.pitch = 1;
  _waypoints.add(t);

  t.yaw = 1;
  t.pitch = -1;
  _waypoints.add(t);

  // subs
  initSubs(KITE_CONTROLLER_SUBS);

  DRONE_PARAM_SUB *sub;

  sub = &_subs[KITE_CONTROLLER_SUB_TURN_RATE_E];
  sub->addrParam = KITE_CONTROLLER_SUB_TURN_RATE_ADDR;
  sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, KITE_CONTROLLER_SUB_TURN_RATE);
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
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, KITE_CONTROLLER_PARAM_LEFT);
  setParamName(FPSTR(STRING_LEFT), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

  param = &_params[KITE_CONTROLLER_PARAM_RIGHT_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, KITE_CONTROLLER_PARAM_RIGHT);
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
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
  param->data.f[0] = -1;
  param->data.f[1] = 1;
  param->data.f[2] = 20;

  param = &_params[KITE_CONTROLLER_PARAM_DISTANCE_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, KITE_CONTROLLER_PARAM_DISTANCE);
  setParamName(FPSTR(STRING_DISTANCE), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
  param->data.f[0] = 0;
  param->data.f[1] = 1;

  param = &_params[KITE_CONTROLLER_PARAM_VECTOR_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, KITE_CONTROLLER_PARAM_VECTOR);
  setParamName(FPSTR(STRING_VECTOR), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 16);

  param = &_params[KITE_CONTROLLER_PARAM_TARGET_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, KITE_CONTROLLER_PARAM_TARGET);
  setParamName(FPSTR(STRING_TARGET), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
  param->data.f[0] = 0;
  param->data.f[1] = 40;

  param = &_params[KITE_CONTROLLER_PARAM_WAYPOINT_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, KITE_CONTROLLER_PARAM_WAYPOINT);
  setParamName(FPSTR(STRING_WAYPOINT), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
  param->data.f[0] = 0;
  param->data.f[1] = 0;
  param->data.f[2] = 0;

  param = &_params[KITE_CONTROLLER_PARAM_PID_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, KITE_CONTROLLER_PARAM_PID);
  setParamName(FPSTR(STRING_PID), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
  param->data.f[0] = 0.03;
  param->data.f[1] = 0;
  param->data.f[2] = 0;

  param = &_params[KITE_CONTROLLER_PARAM_SHAPE_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, KITE_CONTROLLER_PARAM_SHAPE);
  setParamName(FPSTR(STRING_SHAPE), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
  param->data.f[0] = 50;
  param->data.f[1] = 20;
  param->data.f[2] = 5;
}

void KiteControllerModule::selectWaypoint(uint8_t n) {
  if (n <0 || n >= _waypoints.size()) {
    Serial.println("Invalid waypoint number");
    return;
  }

  KITE_CONTROLLER_MODULE_WAYPOINT t = _waypoints.get(n);
  _waypoint = n;
  _wp.yaw = t.yaw * _params[KITE_CONTROLLER_PARAM_SHAPE_E].data.f[0]/2 + _params[KITE_CONTROLLER_PARAM_TARGET_E].data.f[0];
  _wp.pitch = t.pitch * _params[KITE_CONTROLLER_PARAM_SHAPE_E].data.f[1]/2 + _params[KITE_CONTROLLER_PARAM_TARGET_E].data.f[1];
  _wp.radius = _params[KITE_CONTROLLER_PARAM_SHAPE_E].data.f[2];
}

void KiteControllerModule::nextWaypoint() {
  uint8_t n = _waypoint + 1;
  if (n >= _waypoints.size()) n = 0;
  selectWaypoint(n);
}

void KiteControllerModule::checkWaypoint() {
  float delta = abs( 
    SQR( _wp.yaw - _subs[KITE_CONTROLLER_SUB_YAW_E].param.data.f[0] ) + 
    SQR( _wp.pitch - _subs[KITE_CONTROLLER_SUB_PITCH_E].param.data.f[0] ) 
    );
  if (delta <= SQR(_wp.radius)) {
    // reached waypoint
    nextWaypoint();
  }
}

void KiteControllerModule::setup() {
  DroneModule::setup();

  selectWaypoint(0);
}

float KiteControllerModule::manualMode() {
  return _subs[KITE_CONTROLLER_SUB_TURN_RATE_E].param.data.f[0];
}


float KiteControllerModule::autoMode() {
  checkWaypoint();

  float _angToWP = atan2(_wp.yaw - _subs[KITE_CONTROLLER_SUB_YAW_E].param.data.f[0], _wp.pitch - _subs[KITE_CONTROLLER_SUB_PITCH_E].param.data.f[0]);  

  float err = shortestSignedDistanceBetweenCircularValues(radiansToDegrees(_roll), radiansToDegrees(_angToWP));

  float P = _params[KITE_CONTROLLER_PARAM_PID_E].data.f[0] * err;

  return P;
}


void KiteControllerModule::loop() {
  DroneModule::loop();
  if (!_setupDone) return;

  // estimate _roll from motion
  float delta = sqrt( 
    SQR(_subs[KITE_CONTROLLER_SUB_YAW_E].param.data.f[0] - _lastPos[0] ) + 
    SQR(_subs[KITE_CONTROLLER_SUB_PITCH_E].param.data.f[0] - _lastPos[1]) 
    );

  // make sure we've moved far enough to comfortably exceed the noise floor
  if (delta > 0.5) {
    // calc roll as deviation from straight up
    _roll = atan2(_subs[KITE_CONTROLLER_SUB_YAW_E].param.data.f[0] - _lastPos[0], _subs[KITE_CONTROLLER_SUB_PITCH_E].param.data.f[0] - _lastPos[1]);

    // update last pos
    _lastPos[0] = _subs[KITE_CONTROLLER_SUB_YAW_E].param.data.f[0];
    _lastPos[1] = _subs[KITE_CONTROLLER_SUB_PITCH_E].param.data.f[0];
  }

  
  // get turnRate from mode
  float tr = 0;
  switch(_params[KITE_CONTROLLER_PARAM_MODE_E].data.uint8[0]) {
    case KITE_CONTROLLER_MODE_MANUAL: tr = manualMode();  break;
    case KITE_CONTROLLER_MODE_AUTOMATIC: tr = autoMode();  break;
  }
  
  // get params
  float trim = _params[KITE_CONTROLLER_PARAM_TRIM_E].data.f[0];
  float mmin = _params[KITE_CONTROLLER_PARAM_LIMITS_E].data.f[0];
  float mmax = _params[KITE_CONTROLLER_PARAM_LIMITS_E].data.f[1];

  // update payout
  if (_payout != _params[KITE_CONTROLLER_PARAM_DISTANCE_E].data.f[0]) {
    // workout max rate
    float lr = loopRate();
    //Serial.print("loopRate: "); Serial.println(lr);
    float payoutRate = _params[KITE_CONTROLLER_PARAM_DISTANCE_E].data.f[1] / lr;
    //Serial.print("payoutRate: "); Serial.println(payoutRate);
    // calc and constrain delta
    float payoutDelta = constrain(_params[KITE_CONTROLLER_PARAM_DISTANCE_E].data.f[0] - _payout, -payoutRate, payoutRate);
    _payout += payoutDelta;
    //Serial.print("payoutDelta: "); Serial.println(payoutDelta);
    //Serial.print("_payout: "); Serial.println(_payout);
  }

  // constrain payout
  _payout = constrain(_payout, 0, _params[KITE_CONTROLLER_PARAM_LIMITS_E].data.f[2]);

  // merge into state vector
  float state[4];
  state[0] = _subs[KITE_CONTROLLER_SUB_YAW_E].param.data.f[0];
  state[1] = _subs[KITE_CONTROLLER_SUB_PITCH_E].param.data.f[0];
  state[2] = _payout;
  state[3] = radiansToDegrees(_roll);


  float left = 0;
  float right = 0;

  // set left and right based on turnRate
  // right is considered the master and responds direcrly to turnRate.  Left is the opposite
  right = tr;
  left = -tr;

  // constrain left/right to limits
  left = constrain(left, mmin, mmax);
  right = constrain(right, mmin, mmax);

  // old lerp
  //left = mmin + (mmax - mmin) * (left + 1) / 2;
  //right = mmin + (mmax - mmin) * (right + 1) / 2;

  // apply trim to right motor
  right += trim;

  // add payout distance to both motors
  left += _payout;
  right += _payout;

  updateAndPublishParam(&_params[KITE_CONTROLLER_PARAM_LEFT_E], (uint8_t*)&left, sizeof(left));

  updateAndPublishParam(&_params[KITE_CONTROLLER_PARAM_RIGHT_E], (uint8_t*)&right, sizeof(right));

  updateAndPublishParam(&_params[KITE_CONTROLLER_PARAM_VECTOR_E], (uint8_t*)&state, sizeof(state));

  // publish waypoint info
  float wt[3] = { _wp.yaw, _wp.pitch, _wp.radius };
  updateAndPublishParam(&_params[KITE_CONTROLLER_PARAM_WAYPOINT_E], (uint8_t*)&wt, sizeof(wt));
}
