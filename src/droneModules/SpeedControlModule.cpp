#include "SpeedControlModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

SpeedControlModule::SpeedControlModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(SPEED_CONTROL_STR_SPEED_CONTROL));

   // subs
   initSubs(SPEED_CONTROL_SUBS);

   _subs[SPEED_CONTROL_SUB_DISTANCE_E].addrParam = SPEED_CONTROL_SUB_DISTANCE_ADDR;
   _subs[SPEED_CONTROL_SUB_DISTANCE_E].param.param = SPEED_CONTROL_SUB_DISTANCE;
   _subs[SPEED_CONTROL_SUB_DISTANCE_E].param.name = FPSTR(STRING_DISTANCE);
   _subs[SPEED_CONTROL_SUB_DISTANCE_E].param.nameLen = sizeof(STRING_DISTANCE);

   // pubs
   initParams(SPEED_CONTROL_PARAM_ENTRIES);

   _params[SPEED_CONTROL_PARAM_LIMITS_E].param = SPEED_CONTROL_PARAM_LIMITS;
   _params[SPEED_CONTROL_PARAM_LIMITS_E].name = FPSTR(STRING_LIMITS);
   _params[SPEED_CONTROL_PARAM_LIMITS_E].nameLen = sizeof(STRING_LIMITS);
   _params[SPEED_CONTROL_PARAM_LIMITS_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   _params[SPEED_CONTROL_PARAM_LIMITS_E].data.f[0] = 0.2; // just below deadband
   _params[SPEED_CONTROL_PARAM_LIMITS_E].data.f[1] = 1;

   _params[SPEED_CONTROL_PARAM_THRESHOLD_E].param = SPEED_CONTROL_PARAM_THRESHOLD;
   _params[SPEED_CONTROL_PARAM_THRESHOLD_E].name = FPSTR(STRING_THRESHOLD);
   _params[SPEED_CONTROL_PARAM_THRESHOLD_E].nameLen = sizeof(STRING_THRESHOLD);
   _params[SPEED_CONTROL_PARAM_THRESHOLD_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[SPEED_CONTROL_PARAM_THRESHOLD_E].data.f[0] = 3;  // 3 meters

   _params[SPEED_CONTROL_PARAM_SPEED_E].param = SPEED_CONTROL_PARAM_SPEED;
   _params[SPEED_CONTROL_PARAM_SPEED_E].name = FPSTR(STRING_SPEED);
   _params[SPEED_CONTROL_PARAM_SPEED_E].nameLen = sizeof(STRING_SPEED);
   _params[SPEED_CONTROL_PARAM_SPEED_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
}


DEM_NAMESPACE* SpeedControlModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(SPEED_CONTROL_STR_SPEED_CONTROL,0,true);
}

void SpeedControlModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_DISTANCE, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$distance"), DRONE_LINK_MSG_TYPE_FLOAT, pha);

  dem->registerCommand(ns, STRING_LIMITS, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_THRESHOLD, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_SPEED, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}


void SpeedControlModule::update() {
  if (!_setupDone) return;

  // calc and publish new speed

  // check we've received valid distance to go
  if (!_subs[SPEED_CONTROL_SUB_DISTANCE_E].received) return;

  // local shortcuts
  float d = _subs[SPEED_CONTROL_SUB_DISTANCE_E].param.data.f[0];
  float smin = _params[SPEED_CONTROL_PARAM_LIMITS_E].data.f[0];
  float smax = _params[SPEED_CONTROL_PARAM_LIMITS_E].data.f[1];
  float t = _params[SPEED_CONTROL_PARAM_THRESHOLD_E].data.f[0];
  float speed = 0;

  if (d > t) {
    speed = smax;
  } else {
    // lerp
    speed = (d/t) * (smax-smin) + smin;
  }

  updateAndPublishParam(&_params[SPEED_CONTROL_PARAM_SPEED_E], (uint8_t*)&speed, sizeof(speed));
}
