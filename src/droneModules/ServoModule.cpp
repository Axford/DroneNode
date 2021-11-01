#include "ServoModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

ServoModule::ServoModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(SERVO_STR_SERVO));
   //_pins[0] = 0;
   //_limits[0] = -1;
   //_limits[1] = 1;

   // subs
   initSubs(SERVO_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[SERVO_SUB_POSITION_E];
   sub->addrParam = SERVO_SUB_POSITION_ADDR;
   sub->param.param = SERVO_SUB_POSITION;
   setParamName(FPSTR(STRING_POSITION), &sub->param);

   // pubs
   initParams(SERVO_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[SERVO_PARAM_PINS_E];
   param->param = SERVO_PARAM_PINS;
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 3);
   _params[SERVO_PARAM_PINS_E].data.uint8[0] = 0;

   param = &_params[SERVO_PARAM_LIMITS_E];
   param->param = SERVO_PARAM_LIMITS;
   setParamName(FPSTR(STRING_LIMITS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   _params[SERVO_PARAM_LIMITS_E].data.f[0] = -1;
   _params[SERVO_PARAM_LIMITS_E].data.f[1] = 1;

   param = &_params[SERVO_PARAM_MAP_E];
   param->param = SERVO_PARAM_MAP;
   setParamName(FPSTR(STRING_MAP), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 16);
   _params[SERVO_PARAM_MAP_E].data.f[0] = 0;
   _params[SERVO_PARAM_MAP_E].data.f[1] = 60;
   _params[SERVO_PARAM_MAP_E].data.f[2] = 110;
   _params[SERVO_PARAM_MAP_E].data.f[3] = 180;

   param = &_params[SERVO_PARAM_CENTRE_E];
   param->param = SERVO_PARAM_CENTRE;
   setParamName(FPSTR(STRING_CENTRE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[SERVO_PARAM_CENTRE_E].data.f[0] = 90;

   param = &_params[SERVO_PARAM_OUTPUT_E];
   param->param = SERVO_PARAM_OUTPUT;
   setParamName(FPSTR(STRING_OUTPUT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[SERVO_PARAM_OUTPUT_E].data.f[0] = 90;
}


DEM_NAMESPACE* ServoModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(SERVO_STR_SERVO,0,true);
}

void ServoModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_POSITION, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$position"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_PINS, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_LIMITS, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_MAP, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_CENTRE, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}


void ServoModule::setup() {
  DroneModule::setup();

  if (_params[SERVO_PARAM_PINS_E].data.uint8[0] > 0) {
    _servo.setPeriodHertz(50);// Standard 50hz servo
    _servo.attach(_params[SERVO_PARAM_PINS_E].data.uint8[0], 500, 2400);

    // init servo
    update();

  } else {
    Log.errorln(F("Undefined pin %d"), _params[SERVO_PARAM_PINS_E].data.uint8[0]);
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
  _servo.write(pos);

  float f = pos;
  updateAndPublishParam(&_params[SERVO_PARAM_OUTPUT_E], (uint8_t*)&f, sizeof(f));
}
