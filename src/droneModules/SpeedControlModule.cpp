#include "SpeedControlModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

SpeedControlModule::SpeedControlModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(SPEED_CONTROL_STR_SPEED_CONTROL));

   // subs
   initSubs(SPEED_CONTROL_SUBS);

   _subs[SPEED_CONTROL_SUB_DISTANCE_E].addrParam = SPEED_CONTROL_SUB_DISTANCE_ADDR;
   _subs[SPEED_CONTROL_SUB_DISTANCE_E].param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SPEED_CONTROL_SUB_DISTANCE);
   _subs[SPEED_CONTROL_SUB_DISTANCE_E].param.name = FPSTR(STRING_DISTANCE);
   _subs[SPEED_CONTROL_SUB_DISTANCE_E].param.nameLen = sizeof(STRING_DISTANCE);

   // pubs
   initParams(SPEED_CONTROL_PARAM_ENTRIES);

   _params[SPEED_CONTROL_PARAM_LIMITS_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SPEED_CONTROL_PARAM_LIMITS);
   _params[SPEED_CONTROL_PARAM_LIMITS_E].name = FPSTR(STRING_LIMITS);
   _params[SPEED_CONTROL_PARAM_LIMITS_E].nameLen = sizeof(STRING_LIMITS);
   _params[SPEED_CONTROL_PARAM_LIMITS_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   _params[SPEED_CONTROL_PARAM_LIMITS_E].data.f[0] = 0.2; // just below deadband
   _params[SPEED_CONTROL_PARAM_LIMITS_E].data.f[1] = 1;

   _params[SPEED_CONTROL_PARAM_THRESHOLD_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SPEED_CONTROL_PARAM_THRESHOLD);
   _params[SPEED_CONTROL_PARAM_THRESHOLD_E].name = FPSTR(STRING_THRESHOLD);
   _params[SPEED_CONTROL_PARAM_THRESHOLD_E].nameLen = sizeof(STRING_THRESHOLD);
   _params[SPEED_CONTROL_PARAM_THRESHOLD_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[SPEED_CONTROL_PARAM_THRESHOLD_E].data.f[0] = 3;  // 3 meters

   _params[SPEED_CONTROL_PARAM_SPEED_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, SPEED_CONTROL_PARAM_SPEED);
   _params[SPEED_CONTROL_PARAM_SPEED_E].name = FPSTR(STRING_SPEED);
   _params[SPEED_CONTROL_PARAM_SPEED_E].nameLen = sizeof(STRING_SPEED);
   _params[SPEED_CONTROL_PARAM_SPEED_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
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
