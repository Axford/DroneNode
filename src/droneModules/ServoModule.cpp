#include "ServoModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "DroneSystem.h"

ServoModule::ServoModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(SERVO_STR_SERVO));
   //_pins[0] = 0;
   //_limits[0] = -1;
   //_limits[1] = 1;

   // subs
   initSubs(SERVO_SUBS);
   _targetPos = 90;
   _startPos = 90;
   _currentPos = 90;
   _startTime = 0;

   DRONE_PARAM_SUB *sub;

   sub = &_subs[SERVO_SUB_POSITION_E];
   sub->addrParam = SERVO_SUB_POSITION_ADDR;
   // set to high priority so we can monitor its value easily from the server
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, SERVO_SUB_POSITION);
   setParamName(FPSTR(STRING_POSITION), &sub->param);

   // pubs
   initParams(SERVO_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[SERVO_PARAM_PINS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SERVO_PARAM_PINS);
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 3);
   _params[SERVO_PARAM_PINS_E].data.uint8[0] = 0;

   param = &_params[SERVO_PARAM_LIMITS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SERVO_PARAM_LIMITS);
   setParamName(FPSTR(STRING_LIMITS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   _params[SERVO_PARAM_LIMITS_E].data.f[0] = 90;  // 90 degrees per second rate limit
   _params[SERVO_PARAM_LIMITS_E].data.f[1] = 0;  // ignored

   param = &_params[SERVO_PARAM_MAP_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SERVO_PARAM_MAP);
   setParamName(FPSTR(STRING_MAP), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 16);
   _params[SERVO_PARAM_MAP_E].data.f[0] = 0;
   _params[SERVO_PARAM_MAP_E].data.f[1] = 60;
   _params[SERVO_PARAM_MAP_E].data.f[2] = 110;
   _params[SERVO_PARAM_MAP_E].data.f[3] = 180;

   param = &_params[SERVO_PARAM_CENTRE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SERVO_PARAM_CENTRE);
   setParamName(FPSTR(STRING_CENTRE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[SERVO_PARAM_CENTRE_E].data.f[0] = 0;

   param = &_params[SERVO_PARAM_OUTPUT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, SERVO_PARAM_OUTPUT);
   setParamName(FPSTR(STRING_OUTPUT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[SERVO_PARAM_OUTPUT_E].data.f[0] = _targetPos;
}


void ServoModule::setup() {
  DroneModule::setup();

  if (_ds->requestPin(_params[SERVO_PARAM_PINS_E].data.uint8[0], DRONE_SYSTEM_PIN_CAP_OUTPUT, this)) {
    _servo.setPeriodHertz(50);// Standard 50hz servo
    _servo.attach(_params[SERVO_PARAM_PINS_E].data.uint8[0], 500, 2400);

    // init servo
    update();

  } else {
    Log.errorln(F("Unable to register pin %d"), _params[SERVO_PARAM_PINS_E].data.uint8[0]);
    disable();
  }
}

void ServoModule::loop() {
  DroneModule::loop();

  long loopTime = millis();


  if (_currentPos != _targetPos) {

    // total transition time
    float dt = (loopTime - _startTime) / 1000.0;
    //if (dt > 1) dt = 1;

    // calc rate limit - maximum degrees we can have moved since start
    float lim = _params[SERVO_PARAM_LIMITS_E].data.f[0] * dt;
    //if (lim == 0) { lim = 0.1; } // ensure we don't stall in a fast update loop

    // apply rate limit
    float err = _targetPos - _startPos;
    if (err > lim) { err = lim; }
    if (err < -lim) { err = -lim; };

    // update _currentPos
    _currentPos = _startPos + err;
  }

  updateAndPublishParam(&_params[SERVO_PARAM_OUTPUT_E], (uint8_t*)&_currentPos, sizeof(_currentPos));

  // reinforce
  // TODO: this seems to be required, but no idea why
  _servo.write(_currentPos);
}



void ServoModule::update() {
  if (_error > 0 || !_setupDone) return;

  float v = _subs[SERVO_SUB_POSITION_E].param.data.f[0];
  // limit range
  if (v > 1) v = 1;
  if (v< -1) v = -1;

  // remap -1 to 1 into _limits[0] to _limits[1]
  //v = (v + 1) * (_params[SERVO_PARAM_LIMITS_E].data.f[1] - _params[SERVO_PARAM_LIMITS_E].data.f[0]) / (2) + _params[SERVO_PARAM_LIMITS_E].data.f[0];

  // apply bezier map curve
  float t = (v/2) + 0.5;
  float t2 = t * t;
  float t3 = t2 * t;
  float mt = 1 - t;
  float mt2 = mt * mt;
  float mt3 = mt2 * mt;
  v = _params[SERVO_PARAM_MAP_E].data.f[0] * mt3 +
      _params[SERVO_PARAM_MAP_E].data.f[1] * 3 * mt2 * t +
      _params[SERVO_PARAM_MAP_E].data.f[2] * 3 * mt * t2 +
      _params[SERVO_PARAM_MAP_E].data.f[3] * t3;

  int pos = v + _params[SERVO_PARAM_CENTRE_E].data.f[0];
  // limits
  if (pos > 180) pos = 180;
  if (pos < 0) pos = 0;

  //_servo.write(pos);
  if (pos != _targetPos) {
    _targetPos = pos;
    _startPos = _servo.read(); // in case we're not at the new position yet
    _startTime = millis();
  }
}
