#include "StatusModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"

StatusModule::StatusModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(STATUS_STR_STATUS));

   // set default interval to 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   // subs
   initSubs(STATUS_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[STATUS_SUB_SUB1_E];
   sub->addrParam = STATUS_SUB_SUB1_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, STATUS_SUB_SUB1);
   setParamName(FPSTR(STRING_SUB1), &sub->param);

   sub = &_subs[STATUS_SUB_SUB2_E];
   sub->addrParam = STATUS_SUB_SUB2_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, STATUS_SUB_SUB2);
   setParamName(FPSTR(STRING_SUB2), &sub->param);

   sub = &_subs[STATUS_SUB_SUB3_E];
   sub->addrParam = STATUS_SUB_SUB3_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, STATUS_SUB_SUB3);
   setParamName(FPSTR(STRING_SUB3), &sub->param);

   sub = &_subs[STATUS_SUB_SUB4_E];
   sub->addrParam = STATUS_SUB_SUB4_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, STATUS_SUB_SUB4);
   setParamName(FPSTR(STRING_SUB4), &sub->param);


   // pubs
   initParams(STATUS_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[STATUS_PARAM_SCENE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, STATUS_PARAM_SCENE);
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
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, STATUS_PARAM_VALUE1);
   setParamName(FPSTR(STRING_VALUE1), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[STATUS_PARAM_VALUE1_E].data.f[0] = 0.5;

   param = &_params[STATUS_PARAM_VALUE2_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, STATUS_PARAM_VALUE2);
   setParamName(FPSTR(STRING_VALUE2), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[STATUS_PARAM_VALUE2_E].data.f[0] = 0.5;

   param = &_params[STATUS_PARAM_VALUE3_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, STATUS_PARAM_VALUE3);
   setParamName(FPSTR(STRING_VALUE3), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[STATUS_PARAM_VALUE3_E].data.f[0] = 0.5;

   param = &_params[STATUS_PARAM_VALUE4_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, STATUS_PARAM_VALUE4);
   setParamName(FPSTR(STRING_VALUE4), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[STATUS_PARAM_VALUE4_E].data.f[0] = 0.5;
}



void StatusModule::setup() {
  DroneModule::setup();

}


uint8_t StatusModule::checkThreshold(uint8_t index) {
  // check we have a valid sub address
  if (_subs[STATUS_SUB_SUB1_E + index].addr.node == 0) return 0;

  // check length of values and sub match
  uint8_t len = (_params[STATUS_PARAM_VALUE1_E + index].paramTypeLength & 0xF);
  if ((_subs[STATUS_SUB_SUB1_E + index].param.paramTypeLength & 0xF) != len) return 0;

  // check all sub values are above threshold
  uint8_t ty = (_params[STATUS_PARAM_VALUE1_E + index].paramTypeLength >> 4) & 0x7;
  uint8_t numValues = (len+1) / DRONE_LINK_MSG_TYPE_SIZES[ty];
  boolean v = true;
  for (uint8_t i=0; i<numValues; i++) {
    v = v && (_subs[STATUS_SUB_SUB1_E + index].param.data.f[i] >= _params[STATUS_PARAM_VALUE1_E + index].data.f[i]);
  }

  return v ? 2 : 1;
}

void StatusModule::loop() {
  DroneModule::loop();

  uint8_t newScene[16];
  for (uint8_t i=0; i<16; i++) {
    newScene[i] = _params[STATUS_PARAM_SCENE_E].data.uint8[i];
  }

  // check thresholds
  for (uint8_t i=0; i<4; i++) {
    uint8_t threshold = checkThreshold(i);
    if (threshold == 2) {
      newScene[4 + i*3] = 0;
      newScene[5 + i*3] = 255;
      newScene[6 + i*3] = 0;
    } else if (threshold == 1) {
      newScene[4 + i*3] = 255;
      newScene[5 + i*3] = 0;
      newScene[6 + i*3] = 0;
    } else {
      newScene[4 + i*3] = 0;
      newScene[5 + i*3] = 0;
      newScene[6 + i*3] = 0;
    }
  }

  updateAndPublishParam(&_params[STATUS_PARAM_SCENE_E], (uint8_t*)&newScene, sizeof(newScene));
}
