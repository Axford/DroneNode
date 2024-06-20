#include "DigitalModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"
#include "DroneSystem.h"

DigitalModule::DigitalModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(DIGITAL_STR_DIGITAL));

   // set default interval to 100
   // @default interval = 100
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 100;

   // subs
   initSubs(DIGITAL_SUBS);

   DRONE_PARAM_SUB *sub;


   // pubs
   initParams(DIGITAL_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[DIGITAL_PARAM_PINS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, DIGITAL_PARAM_PINS);
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[DIGITAL_PARAM_PINS_E].data.uint8[0] = 0;


   param = &_params[DIGITAL_PARAM_LIMITS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, DIGITAL_PARAM_LIMITS);
   setParamName(FPSTR(STRING_LIMITS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
      // @default limits=0,1
   _params[DIGITAL_PARAM_LIMITS_E].data.f[0] = 0;
   _params[DIGITAL_PARAM_LIMITS_E].data.f[1] = 1;

   param = &_params[DIGITAL_PARAM_INPUT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, DIGITAL_PARAM_INPUT);
   setParamName(FPSTR(STRING_INPUT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[DIGITAL_PARAM_INPUT_E].data.f[0] = 0;
}


void DigitalModule::setup() {
  DroneModule::setup();

  if (_ds->requestPin(_params[DIGITAL_PARAM_PINS_E].data.uint8[0], DRONE_SYSTEM_PIN_CAP_INPUT, this)) {
    // set as input and enable pullup
    pinMode(_params[DIGITAL_PARAM_PINS_E].data.uint8[0], INPUT_PULLUP);

  } else {
    Log.errorln(F("[AM.s] Digital pin unavailable %u"), _id);
    setError(1);
    disable();
  }
}


void DigitalModule::loop() {
  DroneModule::loop();

  boolean raw = digitalRead(_params[DIGITAL_PARAM_PINS_E].data.uint8[0]);

  // map using limits
  float f = raw ? _params[DIGITAL_PARAM_LIMITS_E].data.f[1] : _params[DIGITAL_PARAM_LIMITS_E].data.f[0];

  // publish new values
  updateAndPublishParam(&_params[DIGITAL_PARAM_INPUT_E], (uint8_t*)&f, sizeof(f));
}
