#include "DepthModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"
#include "DroneSystem.h"

// @type Depth

DepthModule::DepthModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(DEPTH_STR_DEPTH));

   _logPos[0] = 0;
   _logPos[1] = 0;

   // set default interval to 1000
   // @default interval = 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   // subs
   initSubs(DEPTH_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[DEPTH_SUB_LOCATION_E];
   sub->addrParam = DEPTH_SUB_LOCATION_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, DEPTH_SUB_LOCATION);
   setParamName(FPSTR(STRING_LOCATION), &sub->param);


   // pubs
   initParams(DEPTH_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[DEPTH_PARAM_PINS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, DEPTH_PARAM_PINS);
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 2);
   _params[DEPTH_PARAM_PINS_E].data.uint8[0] = 0;

   param = &_params[DEPTH_PARAM_SPEED_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, DEPTH_PARAM_SPEED);
   setParamName(FPSTR(STRING_PWM_CHANNEL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
      // @default speed=1480
   _params[DEPTH_PARAM_SPEED_E].data.f[0] = 1480;

   param = &_params[DEPTH_PARAM_LIMITS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, DEPTH_PARAM_LIMITS);
   setParamName(FPSTR(STRING_LIMITS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
      // @default limits=0.25, 10
   _params[DEPTH_PARAM_LIMITS_E].data.f[0] = 0.25;
   _params[DEPTH_PARAM_LIMITS_E].data.f[1] = 10;

   param = &_params[DEPTH_PARAM_DEPTH_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, DEPTH_PARAM_DEPTH);
   setParamName(FPSTR(STRING_DEPTH), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[DEPTH_PARAM_DEPTH_E].data.f[0] = 0;

   param = &_params[DEPTH_PARAM_LOG_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, DEPTH_PARAM_LOG);
   setParamName(FPSTR(STRING_LOG), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[DEPTH_PARAM_LOG_E].data.f[0] = 0;
   _params[DEPTH_PARAM_LOG_E].data.f[1] = 0;
   _params[DEPTH_PARAM_LOG_E].data.f[2] = 0;

   param = &_params[DEPTH_PARAM_DISTANCE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, DEPTH_PARAM_DISTANCE);
   setParamName(FPSTR(STRING_PWM_CHANNEL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[DEPTH_PARAM_DISTANCE_E].data.f[0] = 0;
}


void DepthModule::setup() {
  DroneModule::setup();

  if (_ds->requestPin(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_TRIGGER], DRONE_SYSTEM_PIN_CAP_OUTPUT, this) &&
      _ds->requestPin(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_ECHO], DRONE_SYSTEM_PIN_CAP_INPUT, this)) {

    // set trigger to output
    pinMode(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_TRIGGER], OUTPUT);
    digitalWrite(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_TRIGGER], LOW);

    // set echo to input
    pinMode(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_ECHO], INPUT);

  } else {
    Log.errorln(F("[DM.s] Pins unavailable %u"), _id);
    setError(1);
    disable();
  }
}


void DepthModule::loop() {
  DroneModule::loop();

  // calc timeout value in microseconds
  // 2 * 1000000 * max measurable value / speed of sound
  unsigned long timeout = 10 * 2000000 * _params[DEPTH_PARAM_LIMITS_E].data.f[1] / _params[DEPTH_PARAM_SPEED_E].data.f[0];

  Serial.print("[DM.l] timeout: ");
  Serial.println(timeout);

  // send a pulse
  digitalWrite(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_TRIGGER], HIGH);
  delayMicroseconds(20);
  digitalWrite(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_TRIGGER], LOW);

  // wait for return echo and calcuate duration
  const unsigned long duration= pulseIn(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_ECHO], HIGH, timeout);

  // convert to distance (in meters)
  // Need to half the duration as it represents twice the depth
  float d = _params[DEPTH_PARAM_SPEED_E].data.f[0] * duration / 2000000;

  // publish new depth value
  updateAndPublishParam(&_params[DEPTH_PARAM_DEPTH_E], (uint8_t*)&d, sizeof(d));


  // check how far we've moved since last log entry
  // _logPos
  float dist = calculateDistanceBetweenCoordinates(
    _logPos[0],
    _logPos[1],
    _subs[DEPTH_SUB_LOCATION_E].param.data.f[0],
    _subs[DEPTH_SUB_LOCATION_E].param.data.f[1]
  );

  if (dist > _params[DEPTH_PARAM_DISTANCE_E].data.f[0]) {
    // create composite log param
    float log[3];
    log[0] = _subs[DEPTH_SUB_LOCATION_E].param.data.f[0];
    log[1] = _subs[DEPTH_SUB_LOCATION_E].param.data.f[1];
    log[2] = d;

    _logPos[0] = log[0];
    _logPos[1] = log[1];

    updateAndPublishParam(&_params[DEPTH_PARAM_LOG_E], (uint8_t*)&log, sizeof(log));
  }

  if(duration==0){
    Serial.println("[DM.l] Warning: no pulse from depth sensor");
  } else {
    Serial.print("[DM.l] Depth: ");
    Serial.print(d);
    Serial.print("m, raw: ");
    Serial.print(duration);
  }
}
