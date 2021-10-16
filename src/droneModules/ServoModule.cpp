#include "ServoModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

ServoModule::ServoModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(SERVO_STR_SERVO));
   _pins[0] = 0;
   _limits[0] = -1;
   _limits[1] = 1;

   // subs
   initSubs(SERVO_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[SERVO_SUB_POSITION_E];
   sub->addrParam = SERVO_SUB_POSITION_ADDR;
   sub->param.param = SERVO_SUB_POSITION;
   setParamName(FPSTR(STRING_POSITION), &sub->param);
}



void ServoModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

  // limits
  if (obj.containsKey(STRING_LIMITS)) {
    Log.noticeln(F("[ServoModule.loadConfiguration]  Read limits..."));
    JsonArray array = obj[STRING_LIMITS].as<JsonArray>();
    uint8_t i=0;
    for(JsonVariant v : array) {
      if (i < sizeof(_limits))
        _limits[i] = v | _limits[i];
      i++;
    }
  }

  //TODO: fix
  //DroneModule::parsePins(obj, _pins, (uint8_t)sizeof(_pins));
}


void ServoModule::setup() {
  DroneModule::setup();

  if (_pins[0] > 0) {
    _servo.setPeriodHertz(50);// Standard 50hz servo
    _servo.attach(_pins[0], 500, 2400);

    // init servo
    update();

  } else {
    Log.errorln(F("Undefined pin %d"), _pins[0]);
    disable();
  }
}

void ServoModule::loop() {
  DroneModule::loop();

  // reinforce
  // TODO: this seems to be required, but no idea why
  _servo.write(_servo.read());
}



void ServoModule::update() {
  float v = _subs[SERVO_SUB_POSITION_E].param.data.f[0];
  // limit range
  if (v > 1) v = 1;
  if (v< -1) v = -1;

  // remap -1 to 1 into _limits[0] to _limits[1]
  v = (v + 1) * (_limits[1] - _limits[0]) / (2) + _limits[0];

  int pos = (v*90.0) + 90.0;
  // limits
  if (pos > 180) pos = 180;
  if (pos < 0) pos = 0;
  _servo.write(pos);
}
