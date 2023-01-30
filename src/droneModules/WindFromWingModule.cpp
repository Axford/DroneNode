#include "WindFromWingModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"

WindFromWingModule::WindFromWingModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(WIND_FROM_WING_STR_WIND_FROM_WING));

   _dirSample = 0;

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

   param = &_params[WIND_FROM_WING_PARAM_SAMPLES_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WIND_FROM_WING_PARAM_SAMPLES);
   setParamName(FPSTR(STRING_SAMPLES), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
    _params[WIND_FROM_WING_PARAM_SAMPLES_E].data.uint8[0] = 10;
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
  float wingTailPosition = _subs[WIND_FROM_WING_SUB_WING_E].param.data.f[0];  // which orientation is the tail servo in, either -1 or +1
  float wingHeading = _subs[WIND_FROM_WING_SUB_HEADING_E].param.data.f[0];  // heading of the the wing from its onboard compass in world coordinates
  float AOA = _params[WIND_FROM_WING_PARAM_AOA_E].data.f[0];  // angle of attack

  float windAng = wingHeading + AOA * wingTailPosition;  // wind angle is the wing heading offset by the AOA

  //updateAndPublishParam(&_params[WIND_FROM_WING_PARAM_WIND_E], (uint8_t*)&windAng, sizeof(windAng));


  // update moving average (World coordinates)
  // -------------------------------------------

  float w = windAng;
  w = fmod(w, 360);
  if (w < 0) w += 360;

  // update sample count
  if (_dirSample < _params[WIND_FROM_WING_PARAM_SAMPLES_E].data.uint8[0]) _dirSample++;
  if (_dirSample < 1) _dirSample = 1;

  // recalculate w as shortest circular distance from current average... normal moving average won't work otherwise!
  float w1 = shortestSignedDistanceBetweenCircularValues(_params[WIND_FROM_WING_PARAM_WIND_E].data.f[0], w);
  // then add back to current average to get as a angle we can average
  w1 = _params[WIND_FROM_WING_PARAM_WIND_E].data.f[0] + w1;

  // update moving average
  w = ((_params[WIND_FROM_WING_PARAM_WIND_E].data.f[0] * (_dirSample-1)) + w1) / _dirSample;
  
  w = fmod(w, 360);
  if (w < 0) w += 360;

  updateAndPublishParam(&_params[WIND_FROM_WING_PARAM_WIND_E], (uint8_t*)&w, sizeof(w));
}
