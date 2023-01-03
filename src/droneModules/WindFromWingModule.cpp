#include "WindFromWingModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

WindFromWingModule::WindFromWingModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(WIND_FROM_WING_STR_WIND_FROM_WING));

   // subs
   initSubs(WIND_FROM_WING_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[WIND_FROM_WING_SUB_WING_E];
   sub->addrParam = WIND_FROM_WING_SUB_WING_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WIND_FROM_WING_SUB_WING);
   setParamName(FPSTR(STRING_WING), &sub->param);

   sub = &_subs[WIND_FROM_WING_SUB_HEADING_E];
   sub->addrParam = WIND_FROM_WING_SUB_HEADING_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WIND_FROM_WING_SUB_HEADING);
   setParamName(FPSTR(STRING_HEADING), &sub->param);


   // pubs
   initParams(WIND_FROM_WING_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[WIND_FROM_WING_PARAM_AOA_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WIND_FROM_WING_PARAM_AOA);
   setParamName(FPSTR(STRING_AOA), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[WIND_FROM_WING_PARAM_AOA_E].data.f[0] = 15;

   param = &_params[WIND_FROM_WING_PARAM_WIND_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, WIND_FROM_WING_PARAM_WIND);
   setParamName(FPSTR(STRING_WIND), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
}


DEM_NAMESPACE* WindFromWingModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(WIND_FROM_WING_STR_WIND_FROM_WING,0,true);
}

void WindFromWingModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

}



void WindFromWingModule::loop() {
  if (!_setupDone) return;

  // shortcuts
  float w = _subs[WIND_FROM_WING_SUB_WING_E].param.data.f[0];
  float h = _subs[WIND_FROM_WING_SUB_HEADING_E].param.data.f[0];
  float AOA = _params[WIND_FROM_WING_PARAM_AOA_E].data.f[0];

  float windAng = h + AOA * w;

  updateAndPublishParam(&_params[WIND_FROM_WING_PARAM_WIND_E], (uint8_t*)&windAng, sizeof(windAng));
}
