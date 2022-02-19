#include "TankSteerModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../strings.h"
#include "../navMath.h"

TankSteerModule::TankSteerModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm , dem, fs)
 {
   // set type
   setTypeName(FPSTR(TANK_STEER_STR_TANK_STEER));

   // subs
   initSubs(TANK_STEER_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[TANK_STEER_SUB_TURN_RATE_E];
   sub->addrParam = TANK_STEER_SUB_TURN_RATE_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TANK_STEER_SUB_TURN_RATE);
   setParamName(FPSTR(STRING_TURN_RATE), &sub->param);

   sub = &_subs[TANK_STEER_SUB_SPEED_E];
   sub->addrParam = TANK_STEER_SUB_SPEED_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TANK_STEER_SUB_SPEED);
   setParamName(FPSTR(STRING_SPEED), &sub->param);

   sub = &_subs[TANK_STEER_SUB_TRIM_E];
   sub->addrParam = TANK_STEER_SUB_TRIM_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, TANK_STEER_SUB_TRIM);
   setParamName(FPSTR(STRING_TRIM), &sub->param);


   // pubs
   initParams(TANK_STEER_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[TANK_STEER_PARAM_LEFT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, TANK_STEER_PARAM_LEFT);
   setParamName(FPSTR(STRING_LEFT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[TANK_STEER_PARAM_RIGHT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, TANK_STEER_PARAM_RIGHT);
   setParamName(FPSTR(STRING_RIGHT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[TANK_STEER_PARAM_MODE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, TANK_STEER_PARAM_MODE);
   setParamName(FPSTR(STRING_MODE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   param->data.uint8[0] = TANK_STEER_MODE_MANUAL;  // default to manual

   update();  // set defaults
}


DEM_NAMESPACE* TankSteerModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(TANK_STEER_STR_TANK_STEER,0,true);
}

void TankSteerModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_TURN_RATE, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$turnRate"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_SPEED, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$speed"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_TRIM, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$trim"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_MODE, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
}


void TankSteerModule::update() {
  if (!_setupDone) return;

  // update mode
  if (_params[TANK_STEER_PARAM_MODE_E].data.uint8[0] == TANK_STEER_MODE_MANUAL) {
    _subs[TANK_STEER_SUB_TURN_RATE_E].enabled = false;
    _subs[TANK_STEER_SUB_SPEED_E].enabled = false;
  } else {
    _subs[TANK_STEER_SUB_TURN_RATE_E].enabled = true;
    _subs[TANK_STEER_SUB_SPEED_E].enabled = true;
  }

  // calc and publish new speeds

  // local shortcuts
  float x = _subs[TANK_STEER_SUB_TURN_RATE_E].param.data.f[0];
  // limit turnRate
  if (x > 1) x = 1;
  if (x < -1) x = -1;

  float y = _subs[TANK_STEER_SUB_SPEED_E].param.data.f[0];
  // limit speed
  if (y > 1) y = 1;
  if (y < -1) y = -1;

  // use trim as offset to x value
  x += _subs[TANK_STEER_SUB_TRIM_E].param.data.f[0];

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
