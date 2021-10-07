#include "MotorModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"


MotorModule::MotorModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(MOTOR_STR_MOTOR));

   _deadband = 0.3; // +- 0.2
   // TODO: make deadband a configurable value

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
   setParamName(FPSTR(DRONE_STR_SPEED), &sub->param);
}


void MotorModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

  DroneModule::parsePins(obj, _pins, (uint8_t)sizeof(_pins));

  _deadband = obj[DRONE_STR_DEADBAND] | _deadband;

  _PWMChannel = obj[DRONE_STR_PWM_CHANNEL] | _PWMChannel;
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
    disable();
  }
}


void MotorModule::disable() {
  DroneModule::disable();
  _subs[MOTOR_SUB_SPEED_E].param.data.f[0] = 0;
  update();
}


void MotorModule::update() {
  float v = _subs[MOTOR_SUB_SPEED_E].param.data.f[0];

  // limit range
  if (v > 1) v = 1;
  if (v< -1) v = -1;

  // check for deadband
  if (abs(v) < abs(_deadband)) v = 0;

  ledcWrite(_PWMChannel, abs(v)*255);

  digitalWrite(_pins[MOTOR_PIN_A], _subs[MOTOR_SUB_SPEED_E].param.data.f[0] > 0);
  digitalWrite(_pins[MOTOR_PIN_B], _subs[MOTOR_SUB_SPEED_E].param.data.f[0] < 0);
}
