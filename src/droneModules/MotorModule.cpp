#include "MotorModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "DroneSystem.h"

MotorModule::MotorModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(MOTOR_STR_MOTOR));

   // subs
   initSubs(MOTOR_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[MOTOR_SUB_SPEED_E];
   sub->addrParam = MOTOR_SUB_SPEED_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, MOTOR_SUB_SPEED);
   setParamName(FPSTR(STRING_SPEED), &sub->param);

   // pubs
   initParams(MOTOR_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[MOTOR_PARAM_PINS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, MOTOR_PARAM_PINS);
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 3);
   _params[MOTOR_PARAM_PINS_E].data.uint8[0] = 0;

   param = &_params[MOTOR_PARAM_PWMCHANNEL_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, MOTOR_PARAM_PWMCHANNEL);
   setParamName(FPSTR(STRING_PWM_CHANNEL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[MOTOR_PARAM_PWMCHANNEL_E].data.uint8[0] = 15;

   param = &_params[MOTOR_PARAM_LIMITS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, MOTOR_PARAM_LIMITS);
   setParamName(FPSTR(STRING_LIMITS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   _params[MOTOR_PARAM_LIMITS_E].data.f[0] = -1;
   _params[MOTOR_PARAM_LIMITS_E].data.f[1] = 1;

   param = &_params[MOTOR_PARAM_DEADBAND_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, MOTOR_PARAM_DEADBAND);
   setParamName(FPSTR(STRING_DEADBAND), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[MOTOR_PARAM_DEADBAND_E].data.f[0] = 0.3;

}

DEM_NAMESPACE* MotorModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(MOTOR_STR_MOTOR,0,true);
}

void MotorModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_SPEED, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$speed"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_PWM_CHANNEL, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_PINS, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_DEADBAND, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_LIMITS, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}


void MotorModule::setup() {
  DroneModule::setup();

  // register pins
  boolean registered = true;
  for (uint8_t i=0; i<3; i++) {
    registered &= _ds->requestPin(_params[MOTOR_PARAM_PINS_E].data.uint8[i], DRONE_SYSTEM_PIN_CAP_OUTPUT, this);
  }

  if (registered) {
    // configure LED PWM functionalitites
    ledcSetup(_params[MOTOR_PARAM_PWMCHANNEL_E].data.uint8[0], 5000, 8);

    ledcAttachPin(_params[MOTOR_PARAM_PINS_E].data.uint8[MOTOR_PIN_EN], _params[MOTOR_PARAM_PWMCHANNEL_E].data.uint8[0]);
    ledcWrite(_params[MOTOR_PARAM_PWMCHANNEL_E].data.uint8[0], 0);  // turn off at start

    pinMode(_params[MOTOR_PARAM_PINS_E].data.uint8[MOTOR_PIN_A], OUTPUT);
    digitalWrite(_params[MOTOR_PARAM_PINS_E].data.uint8[MOTOR_PIN_A], LOW);

    pinMode(_params[MOTOR_PARAM_PINS_E].data.uint8[MOTOR_PIN_B], OUTPUT);
    digitalWrite(_params[MOTOR_PARAM_PINS_E].data.uint8[MOTOR_PIN_B], LOW);

  } else {
    Log.errorln(F("[MM.s] Pins unavailable %u"), _id);
    setError(1);
    disable();
  }
}


void MotorModule::disable() {
  DroneModule::disable();
  _subs[MOTOR_SUB_SPEED_E].param.data.f[0] = 0;
  update();
}


void MotorModule::update() {
  if (_error > 0 || !_setupDone) return;

  float v = _subs[MOTOR_SUB_SPEED_E].param.data.f[0];

  // limit range
  if (v > 1) v = 1;
  if (v< -1) v = -1;

  // remap -1 to 1 into _limits[0] to _limits[1]
  v = (v + 1) * (_params[MOTOR_PARAM_LIMITS_E].data.f[1] - _params[MOTOR_PARAM_LIMITS_E].data.f[0]) / (2) + _params[MOTOR_PARAM_LIMITS_E].data.f[0];

  // check for deadband
  if (abs(v) < abs(_params[MOTOR_PARAM_DEADBAND_E].data.f[0])) v = 0;

  ledcWrite(_params[MOTOR_PARAM_PWMCHANNEL_E].data.uint8[0], abs(v)*255);

  digitalWrite(_params[MOTOR_PARAM_PINS_E].data.uint8[MOTOR_PIN_A], _subs[MOTOR_SUB_SPEED_E].param.data.f[0] > 0);
  digitalWrite(_params[MOTOR_PARAM_PINS_E].data.uint8[MOTOR_PIN_B], _subs[MOTOR_SUB_SPEED_E].param.data.f[0] < 0);
}
