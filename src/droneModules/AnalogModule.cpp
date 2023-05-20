#include "AnalogModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"
#include "DroneSystem.h"

AnalogModule::AnalogModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(ANALOG_STR_ANALOG));

   // set default interval to 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   // subs
   initSubs(ANALOG_SUBS);

   DRONE_PARAM_SUB *sub;


   // pubs
   initParams(ANALOG_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[ANALOG_PARAM_PINS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ANALOG_PARAM_PINS);
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[ANALOG_PARAM_PINS_E].data.uint8[0] = 0;

   param = &_params[ANALOG_PARAM_RAW_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, ANALOG_PARAM_RAW);
   setParamName(FPSTR(STRING_RAW), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[ANALOG_PARAM_RAW_E].data.uint32[0] = 0;

   param = &_params[ANALOG_PARAM_LIMITS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ANALOG_PARAM_LIMITS);
   setParamName(FPSTR(STRING_LIMITS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   _params[ANALOG_PARAM_LIMITS_E].data.f[0] = 0;
   _params[ANALOG_PARAM_LIMITS_E].data.f[1] = 1;

   param = &_params[ANALOG_PARAM_ANALOG_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, ANALOG_PARAM_ANALOG);
   setParamName(FPSTR(STRING_ANALOG), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[ANALOG_PARAM_ANALOG_E].data.f[0] = 0;
}


void AnalogModule::setup() {
  DroneModule::setup();

  if (_ds->requestPin(_params[ANALOG_PARAM_PINS_E].data.uint8[0], DRONE_SYSTEM_PIN_CAP_ANALOG, this)) {

  } else {
    Log.errorln(F("[AM.s] Analog pin unavailable %u"), _id);
    setError(1);
    disable();
  }
}


void AnalogModule::loop() {
  DroneModule::loop();

  uint32_t raw = analogRead(_params[ANALOG_PARAM_PINS_E].data.uint8[0]);

  // map using limits
  float f = _params[ANALOG_PARAM_LIMITS_E].data.f[0] + (_params[ANALOG_PARAM_LIMITS_E].data.f[1]-_params[ANALOG_PARAM_LIMITS_E].data.f[0]) * (raw/4095.0);

  // publish new values
  updateAndPublishParam(&_params[ANALOG_PARAM_ANALOG_E], (uint8_t*)&f, sizeof(f));
  updateAndPublishParam(&_params[ANALOG_PARAM_RAW_E], (uint8_t*)&raw, sizeof(raw));
}
