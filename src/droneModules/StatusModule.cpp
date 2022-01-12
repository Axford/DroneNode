#include "StatusModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"

StatusModule::StatusModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(STATUS_STR_STATUS));

   // set default interval to 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   // subs
   initSubs(STATUS_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[STATUS_SUB_SUB1_E];
   sub->addrParam = STATUS_SUB_SUB1_ADDR;
   sub->param.param = STATUS_SUB_SUB1;
   setParamName(FPSTR(STRING_SUB1), &sub->param);

   sub = &_subs[STATUS_SUB_SUB2_E];
   sub->addrParam = STATUS_SUB_SUB2_ADDR;
   sub->param.param = STATUS_SUB_SUB2;
   setParamName(FPSTR(STRING_SUB2), &sub->param);

   sub = &_subs[STATUS_SUB_SUB3_E];
   sub->addrParam = STATUS_SUB_SUB3_ADDR;
   sub->param.param = STATUS_SUB_SUB3;
   setParamName(FPSTR(STRING_SUB3), &sub->param);

   sub = &_subs[STATUS_SUB_SUB4_E];
   sub->addrParam = STATUS_SUB_SUB4_ADDR;
   sub->param.param = STATUS_SUB_SUB4;
   setParamName(FPSTR(STRING_SUB4), &sub->param);


   // pubs
   initParams(STATUS_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[STATUS_PARAM_SCENE_E];
   param->param = STATUS_PARAM_SCENE;
   setParamName(FPSTR(STRING_SCENE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 16);
   _params[STATUS_PARAM_SCENE_E].data.uint8[0] = 50;
   _params[STATUS_PARAM_SCENE_E].data.uint8[1] = 0;
   _params[STATUS_PARAM_SCENE_E].data.uint8[2] = 0;
   _params[STATUS_PARAM_SCENE_E].data.uint8[3] = 0;

   _params[STATUS_PARAM_SCENE_E].data.uint8[4] = 0;
   _params[STATUS_PARAM_SCENE_E].data.uint8[5] = 0;
   _params[STATUS_PARAM_SCENE_E].data.uint8[6] = 0;

   _params[STATUS_PARAM_SCENE_E].data.uint8[7] = 0;
   _params[STATUS_PARAM_SCENE_E].data.uint8[8] = 0;
   _params[STATUS_PARAM_SCENE_E].data.uint8[9] = 0;

   _params[STATUS_PARAM_SCENE_E].data.uint8[10] = 0;
   _params[STATUS_PARAM_SCENE_E].data.uint8[11] = 0;
   _params[STATUS_PARAM_SCENE_E].data.uint8[12] = 0;

   _params[STATUS_PARAM_SCENE_E].data.uint8[13] = 0;
   _params[STATUS_PARAM_SCENE_E].data.uint8[14] = 0;
   _params[STATUS_PARAM_SCENE_E].data.uint8[15] = 0;

   param = &_params[STATUS_PARAM_VALUE1_E];
   param->param = STATUS_PARAM_VALUE1;
   setParamName(FPSTR(STRING_VALUE1), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[STATUS_PARAM_VALUE1_E].data.f[0] = 0.5;

   param = &_params[STATUS_PARAM_VALUE2_E];
   param->param = STATUS_PARAM_VALUE2;
   setParamName(FPSTR(STRING_VALUE2), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[STATUS_PARAM_VALUE2_E].data.f[0] = 0.5;

   param = &_params[STATUS_PARAM_VALUE3_E];
   param->param = STATUS_PARAM_VALUE3;
   setParamName(FPSTR(STRING_VALUE3), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[STATUS_PARAM_VALUE3_E].data.f[0] = 0.5;

   param = &_params[STATUS_PARAM_VALUE4_E];
   param->param = STATUS_PARAM_VALUE4;
   setParamName(FPSTR(STRING_VALUE4), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[STATUS_PARAM_VALUE4_E].data.f[0] = 0.5;
}

DEM_NAMESPACE* StatusModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(STATUS_STR_STATUS,0,true);
}

void StatusModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);
  dem->registerCommand(ns, STRING_SUB1, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$sub1"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_SUB2, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$sub2"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_SUB3, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$sub3"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_SUB4, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$sub4"), DRONE_LINK_MSG_TYPE_FLOAT, pha);

  dem->registerCommand(ns, STRING_SCENE, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_VALUE1, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_VALUE2, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_VALUE3, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_VALUE4, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}


void StatusModule::setup() {
  DroneModule::setup();

}


void StatusModule::loop() {
  DroneModule::loop();

  uint8_t newScene[16];
  for (uint8_t i=0; i<16; i++) {
    newScene[i] = _params[STATUS_PARAM_SCENE_E].data.uint8[i];
  }

  // sub1
  if (_subs[STATUS_SUB_SUB1_E].addr.node > 0) {
    if (_subs[STATUS_SUB_SUB1_E].param.data.f[0] >= _params[STATUS_PARAM_VALUE1_E].data.f[0]) {
      newScene[4] = 0;
      newScene[5] = 255;
      newScene[6] = 0;
    } else {
      newScene[4] = 255;
      newScene[5] = 0;
      newScene[6] = 0;
    }
  }


  // sub2
  if (_subs[STATUS_SUB_SUB2_E].addr.node > 0) {
    if (_subs[STATUS_SUB_SUB2_E].param.data.f[0] >= _params[STATUS_PARAM_VALUE2_E].data.f[0]) {
      newScene[7] = 0;
      newScene[8] = 255;
      newScene[9] = 0;
    } else {
      newScene[7] = 255;
      newScene[8] = 0;
      newScene[9] = 0;
    }
  }

  // sub3
  if (_subs[STATUS_SUB_SUB3_E].addr.node > 0) {
    if (_subs[STATUS_SUB_SUB3_E].param.data.f[0] >= _params[STATUS_PARAM_VALUE3_E].data.f[0]) {
      newScene[10] = 0;
      newScene[11] = 255;
      newScene[12] = 0;
    } else {
      newScene[10] = 255;
      newScene[11] = 0;
      newScene[12] = 0;
    }
  }

  // sub4
  if (_subs[STATUS_SUB_SUB4_E].addr.node > 0) {
    if (_subs[STATUS_SUB_SUB4_E].param.data.f[0] >= _params[STATUS_PARAM_VALUE4_E].data.f[0]) {
      newScene[13] = 0;
      newScene[14] = 255;
      newScene[15] = 0;
    } else {
      newScene[13] = 255;
      newScene[14] = 0;
      newScene[15] = 0;
    }
  }


  updateAndPublishParam(&_params[STATUS_PARAM_SCENE_E], (uint8_t*)&newScene, sizeof(newScene));
}
