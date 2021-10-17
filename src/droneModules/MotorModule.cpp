#include "MotorModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

MotorModule::MotorModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem):
  DroneModule ( id, dmm, dlm, dem )
 {
   setTypeName(FPSTR(MOTOR_STR_MOTOR));

   _deadband = 0.3; // +- 0.2

   _limits[0] = -1;
   _limits[1] = 1;

   _pins[0] = 0;
   _pins[1] = 0;
   _pins[2] = 0;

   _PWMChannel = 15;

   // subs
   initSubs(MOTOR_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[MOTOR_SUB_SPEED_E];
   sub->addrParam = MOTOR_SUB_SPEED_ADDR;
   sub->param.param = MOTOR_SUB_SPEED;
   setParamName(FPSTR(STRING_SPEED), &sub->param);
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

  dem->registerCommand(ns, STRING_SPEED, DRONE_LINK_MSG_TYPE_CHAR, ph);
  dem->registerCommand(ns, STRING_PWM_CHANNEL, DRONE_LINK_MSG_TYPE_CHAR, ph);
  dem->registerCommand(ns, STRING_PINS, DRONE_LINK_MSG_TYPE_CHAR, ph);
  dem->registerCommand(ns, STRING_DEADBAND, DRONE_LINK_MSG_TYPE_CHAR, ph);
  dem->registerCommand(ns, STRING_LIMITS, DRONE_LINK_MSG_TYPE_CHAR, ph);
}


void MotorModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

  // TODO: fix
  //DroneModule::parsePins(obj, _pins, (uint8_t)sizeof(_pins));

  _deadband = obj[STRING_DEADBAND] | _deadband;

  // limits
  if (obj.containsKey(STRING_LIMITS)) {
    Log.noticeln(F("[MotorModule.loadConfiguration]  Read limits..."));
    JsonArray array = obj[STRING_LIMITS].as<JsonArray>();
    uint8_t i=0;
    for(JsonVariant v : array) {
      if (i < sizeof(_limits))
        _limits[i] = v | _limits[i];
      i++;
    }
  }

  _PWMChannel = obj[STRING_PWM_CHANNEL] | _PWMChannel;
}


void MotorModule::setup() {
  DroneModule::setup();

  if (_pins[0] > 0) {
    // configure LED PWM functionalitites
    ledcSetup(_PWMChannel, 5000, 8);

    ledcAttachPin(_pins[MOTOR_PIN_EN], _PWMChannel);
    ledcWrite(_PWMChannel, 0);  // turn off at start

    pinMode(_pins[MOTOR_PIN_A], OUTPUT);
    digitalWrite(_pins[MOTOR_PIN_A], LOW);

    pinMode(_pins[MOTOR_PIN_B], OUTPUT);
    digitalWrite(_pins[MOTOR_PIN_B], LOW);

  } else {
    Log.errorln(F("Undefined pins"));
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
  v = (v + 1) * (_limits[1] - _limits[0]) / (2) + _limits[0];

  // check for deadband
  if (abs(v) < abs(_deadband)) v = 0;

  ledcWrite(_PWMChannel, abs(v)*255);

  digitalWrite(_pins[MOTOR_PIN_A], _subs[MOTOR_SUB_SPEED_E].param.data.f[0] > 0);
  digitalWrite(_pins[MOTOR_PIN_B], _subs[MOTOR_SUB_SPEED_E].param.data.f[0] < 0);
}
