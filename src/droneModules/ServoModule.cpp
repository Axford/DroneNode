#include "ServoModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../DroneModuleManager.h"
#include "strings.h"
#include "DroneSystem.h"
#include "../CloudTelemetryConfig.h"
#include <math.h>

// @type           Servo

ServoModule::ServoModule(uint8_t id, DroneSystem* ds):
  ServoModule(id, ds, Registration::Managed) {}

ServoModule::ServoModule(uint8_t id, DroneSystem* ds, Registration registration):
  DroneModule(id, ds, registration), _localControl(registration == Registration::Detached)
 {
   if (!_mgmtParams) return;
   setTypeName(FPSTR(SERVO_STR_SERVO));
   //_pins[0] = 0;
   //_limits[0] = -1;
   //_limits[1] = 1;

   // subs
   initSubs(SERVO_SUBS);
   if (!_subs) return;
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
   if (!_params) return;

   DRONE_PARAM_ENTRY *param;

   param = &_params[SERVO_PARAM_PINS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SERVO_PARAM_PINS);
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   // @default pins=0
   _params[SERVO_PARAM_PINS_E].data.uint8[0] = 0;

   param = &_params[SERVO_PARAM_LIMITS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SERVO_PARAM_LIMITS);
   setParamName(FPSTR(STRING_LIMITS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   // @default limits=90
   _params[SERVO_PARAM_LIMITS_E].data.f[0] = 90;  // 90 degrees per second rate limit
  

   param = &_params[SERVO_PARAM_MAP_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SERVO_PARAM_MAP);
   setParamName(FPSTR(STRING_MAP), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 16);
   // @default map=0,60,110,180
   _params[SERVO_PARAM_MAP_E].data.f[0] = 0;
   _params[SERVO_PARAM_MAP_E].data.f[1] = 60;
   _params[SERVO_PARAM_MAP_E].data.f[2] = 110;
   _params[SERVO_PARAM_MAP_E].data.f[3] = 180;

   param = &_params[SERVO_PARAM_CENTRE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SERVO_PARAM_CENTRE);
   setParamName(FPSTR(STRING_CENTRE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   // @default centre=0
   _params[SERVO_PARAM_CENTRE_E].data.f[0] = 0;

   param = &_params[SERVO_PARAM_OUTPUT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, SERVO_PARAM_OUTPUT);
   setParamName(FPSTR(STRING_OUTPUT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[SERVO_PARAM_OUTPUT_E].data.f[0] = _targetPos;
}


ServoModule::~ServoModule() {
  releaseOutput();
}

configuration::Setting ServoModule::configure(const char* name, const char* value) {
  using configuration::Setting;
  if (!_localControl) return Setting::Unhandled;
  auto* param = getParamEntryByName(name);
  if (!param) return Setting::Unhandled;
  const auto id = getDroneLinkMsgParam(param->paramPriority);
  if (id == SERVO_PARAM_PINS) {
    if (strcmp(value, "32") && strcmp(value, "33")) return Setting::Invalid;
    param->data.uint8[0] = atoi(value);
    return Setting::Handled;
  }
  if (id != SERVO_PARAM_MAP && id != SERVO_PARAM_CENTRE && id != SERVO_PARAM_LIMITS)
    return Setting::Unhandled;
  const unsigned count = id == SERVO_PARAM_MAP ? 4 : 1;
  for (unsigned i = 0; i < count; ++i) {
    char* end;
    const float number = strtof(value, &end);
    const float low = id == SERVO_PARAM_CENTRE ? -180 : 0;
    const float high = id == SERVO_PARAM_LIMITS ? 3600 : 180;
    if (end == value || !isfinite(number) || number < low || number > high ||
        *end != (i + 1 == count ? '\0' : ',')) return Setting::Invalid;
    param->data.f[i] = number;
    value = end + 1;
  }
  return Setting::Handled;
}

bool ServoModule::reserveOutput(uint8_t pin) {
  if (!_ds->requestPin(pin, DRONE_SYSTEM_PIN_CAP_OUTPUT, this)) return false;
  _ownedPin = pin;
  return true;
}

bool ServoModule::attachOutput() {
  if (_ownedPin == 255) return false;
  _servo.setPeriodHertz(50);
  _servo.attach(_ownedPin, 500, 2400);
  return _servo.attached();
}

void ServoModule::stopPwm() {
  if (!_servo.attached()) return;
  if (auto* pwm = pwmFactory(_ownedPin)) pwm->write(0);
  _servo.detach();
  pinMode(_ownedPin, INPUT);
}

void ServoModule::releaseOutput() {
  stopPwm();
  if (_ownedPin != 255) _ds->releasePin(_ownedPin, this);
  _ownedPin = 255;
}

void ServoModule::disable() {
  _armed = false;
  if (_localControl) stopPwm();
  DroneModule::disable();
}

void ServoModule::doShutdown() {
  _armed = false;
  releaseOutput();
  DroneModule::doShutdown();
}

void ServoModule::handleLinkMessage(DroneLinkMsg* msg) {
  // Live actuator commands use the physical console. Retained and delayed mesh
  // writes carry no apply generation and cannot prove operator intent.
  if (!_localControl) {
    DroneModule::handleLinkMessage(msg);
    return;
  }
  if (msg->node() != _dlm->node() || msg->channel() != _id) return;
  if (msg->type() != DRONE_LINK_MSG_TYPE_QUERY && msg->type() != DRONE_LINK_MSG_TYPE_NAMEQUERY) return;
  auto* param = getParamEntryById(msg->param());
  if (param) handleParamMessage(msg, param);
}

void ServoModule::consoleControl(const char* action, Print &output) {
  if (!_localControl || _ownedPin == 255) {
    output.println("servo error=unavailable");
    return;
  }
  if (!strcmp(action, "off")) {
    disable();
  } else if (!strcmp(action, "arm")) {
    if (!_servo.attached()) _armed = true;
  } else if (strcmp(action, "status")) {
    char* end;
    const float position = strtof(action, &end);
    if (!_armed || end == action || *end || !isfinite(position) || position < -1 || position > 1) {
      output.println("servo error=invalid-position-or-inhibited");
      return;
    }
    if (!_servo.attached()) {
      // Motor uses the same hardware timers outside ESP32Servo's allocator.
      bool timerAvailable = false;
      for (int i = 2; i < 4; ++i) {
        if (ESP32PWM::timerCount[i] < 4 &&
            (ESP32PWM::timerFreqSet[i] == -1 || ESP32PWM::timerFreqSet[i] == 50))
          timerAvailable = true;
      }
      for (uint8_t i = 0; i < _dmm->moduleCount(); ++i) {
        auto* type = _dmm->getModuleByIndex(i)->getParamEntryById(DRONE_MODULE_PARAM_TYPE);
        if (type && (type->paramTypeLength & 15) == 4 && !memcmp(type->data.c, "Motor", 5))
          timerAvailable = false;
      }
      if (!timerAvailable || !attachOutput()) {
        disable();
        output.println("servo error=pwm-unavailable");
        return;
      }
      // Start the rate limiter at centre rather than reading a detached channel.
      _servo.write(90);
      _targetPos = _currentPos = _startPos = 90;
      DroneModule::enable();
    }
    _subs[SERVO_SUB_POSITION_E].param.data.f[0] = position;
    update();
  }
  const auto* state = _servo.attached() ? "active" : _armed ? "armed" : "inhibited";
  auto* pwm = _servo.attached() ? pwmFactory(_ownedPin) : nullptr;
  output.printf("servo state=%s pin=%u duty=%lu\n", state, _ownedPin,
                static_cast<unsigned long>(pwm ? pwm->read() : 0));
}

void ServoModule::setup() {
  if (!hotLoadReady()) return;
  DroneModule::setup();
  if (!reserveOutput(_params[SERVO_PARAM_PINS_E].data.uint8[0])) {
    disable();
    return;
  }
  if (_localControl) {
    disable();
    return;
  }
  if (attachOutput()) update();
  else {
    releaseOutput();
    disable();
  }
}

void ServoModule::loop() {
  DroneModule::loop();
  if (!_servo.attached()) return;

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
  if (_error > 0 || !_setupDone || !_servo.attached()) return;

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
