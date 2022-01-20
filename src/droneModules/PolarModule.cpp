#include "PolarModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"

PolarModule::PolarModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(POLAR_STR_POLAR));

   // init polarVals
   for (uint8_t i=0; i<16; i++) _polarVals[i] = 0;
   _startTime = 0;
   _startPos[0] = 0;
   _startPos[1] = 0;

   // set default interval to 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   // subs
   initSubs(POLAR_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[POLAR_SUB_LOCATION_E];
   sub->addrParam = POLAR_SUB_LOCATION_ADDR;
   sub->param.param = POLAR_SUB_LOCATION;
   setParamName(FPSTR(STRING_LOCATION), &sub->param);
   sub->param.data.f[0] =0;
   sub->param.data.f[1] =0;

   sub = &_subs[POLAR_SUB_SOG_E];
   sub->addrParam = POLAR_SUB_SOG_ADDR;
   sub->param.param = POLAR_SUB_SOG;
   setParamName(FPSTR(STRING_SOG), &sub->param);

   sub = &_subs[POLAR_SUB_WIND_E];
   sub->addrParam = POLAR_SUB_WIND_ADDR;
   sub->param.param = POLAR_SUB_WIND;
   setParamName(FPSTR(STRING_WIND), &sub->param);

   sub = &_subs[POLAR_SUB_WIND_SPEED_E];
   sub->addrParam = POLAR_SUB_WIND_SPEED_ADDR;
   sub->param.param = POLAR_SUB_WIND_SPEED;
   setParamName(FPSTR(STRING_WIND_SPEED), &sub->param);

   sub = &_subs[POLAR_SUB_HEADING_E];
   sub->addrParam = POLAR_SUB_HEADING_ADDR;
   sub->param.param = POLAR_SUB_HEADING;
   setParamName(FPSTR(STRING_HEADING), &sub->param);


   // pubs
   initParams(POLAR_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[POLAR_PARAM_TARGET_E];
   param->param = POLAR_PARAM_TARGET;
   setParamName(FPSTR(STRING_PWM_CHANNEL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   param->data.f[0] = 0;
   param->data.f[1] = 0;

   param = &_params[POLAR_PARAM_MODE_E];
   param->param = POLAR_PARAM_MODE;
   setParamName(FPSTR(STRING_PWM_CHANNEL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   param->data.uint8[0] = 0;

   param = &_params[POLAR_PARAM_THRESHOLD_E];
   param->param = POLAR_PARAM_THRESHOLD;
   setParamName(FPSTR(STRING_PWM_CHANNEL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   param->data.f[0] = 1;  // min SOG
   param->data.f[1] = 10;  // acceptable heading deviation

   param = &_params[POLAR_PARAM_POLAR_E];
   param->param = POLAR_PARAM_POLAR;
   setParamName(FPSTR(STRING_POLAR), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);
   for (uint8_t i=0; i<16; i++) param->data.uint8[i] = 0;

   param = &_params[POLAR_PARAM_SAMPLES_E];
   param->param = POLAR_PARAM_SAMPLES;
   setParamName(FPSTR(STRING_PWM_CHANNEL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);
   for (uint8_t i=0; i<16; i++) param->data.uint8[i] = 0;

   param = &_params[POLAR_PARAM_RADIUS_E];
   param->param = POLAR_PARAM_RADIUS;
   setParamName(FPSTR(STRING_PWM_CHANNEL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[0] = 0; // inner
   param->data.f[1] = 0; // mid
   param->data.f[2] = 0; // outer radii

}

DEM_NAMESPACE* PolarModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(POLAR_STR_POLAR,0,true);
}

void PolarModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);
  dem->registerCommand(ns, STRING_LOCATION, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$location"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_SOG, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$SOG"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_WIND, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$wind"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_WIND_SPEED, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$windSpeed"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_HEADING, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$heading"), DRONE_LINK_MSG_TYPE_FLOAT, pha);

  dem->registerCommand(ns, STRING_MODE, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_TARGET, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_THRESHOLD, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_RADIUS, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}


void PolarModule::setup() {
  DroneModule::setup();

}


void PolarModule::loop() {
  DroneModule::loop();


  // publish new depth value
  //updateAndPublishParam(&_params[POLAR_PARAM_POLAR_E], (uint8_t*)&d, sizeof(d));


  // check how far we've moved since start
  /*
  float dist = calculateDistanceBetweenCoordinates(
    _startPos[0],
    _startPos[1]
    _subs[POLAR_SUB_LOCATION_E].param.data.f[0],
    _subs[POLAR_SUB_LOCATION_E].param.data.f[1]
  );
  */

}
