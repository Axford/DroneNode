#include "WaterSensorModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"
#include "DroneSystem.h"

WaterSensorModule::WaterSensorModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(WATER_SENSOR_STR_WATER_SENSOR));

   _triggered = false;
   _triggerTime = 0;

   // set default interval to 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1;

   // subs
   initSubs(WATER_SENSOR_SUBS);

   DRONE_PARAM_SUB *sub;


   // pubs
   initParams(WATER_SENSOR_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[WATER_SENSOR_PARAM_PINS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WATER_SENSOR_PARAM_PINS);
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 2);
   _params[WATER_SENSOR_PARAM_PINS_E].data.uint8[0] = 0;
   _params[WATER_SENSOR_PARAM_PINS_E].data.uint8[1] = 0;

   param = &_params[WATER_SENSOR_PARAM_RAW_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, WATER_SENSOR_PARAM_RAW);
   setParamName(FPSTR(STRING_RAW), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[WATER_SENSOR_PARAM_RAW_E].data.uint32[0] = 0;

   param = &_params[WATER_SENSOR_PARAM_THRESHOLD_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WATER_SENSOR_PARAM_THRESHOLD);
   setParamName(FPSTR(STRING_THRESHOLD), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[WATER_SENSOR_PARAM_THRESHOLD_E].data.uint32[0] = 0;

    param = &_params[WATER_SENSOR_PARAM_ALARM_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, WATER_SENSOR_PARAM_ALARM);
   setParamName(FPSTR(STRING_ALARM), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[WATER_SENSOR_PARAM_ALARM_E].data.f[0] = 0;
}


void WaterSensorModule::setup() {
  DroneModule::setup();

  if (_ds->requestPin(_params[WATER_SENSOR_PARAM_PINS_E].data.uint8[0], DRONE_SYSTEM_PIN_CAP_OUTPUT, this) && 
      _ds->requestPin(_params[WATER_SENSOR_PARAM_PINS_E].data.uint8[1], DRONE_SYSTEM_PIN_CAP_ANALOG, this)) {

    pinMode(_params[WATER_SENSOR_PARAM_PINS_E].data.uint8[0], OUTPUT);
    // set output low to start
    digitalWrite(_params[WATER_SENSOR_PARAM_PINS_E].data.uint8[0], LOW);

  } else {
    Log.errorln(F("[AM.s] Pins unavailable %u"), _id);
    setError(1);
    disable();
  }
}


void WaterSensorModule::loop() {
  DroneModule::loop();

  unsigned long loopTime = millis();

  if (_triggered) {
    if (loopTime > _triggerTime + WATER_SENSOR_SAMPLE_TRIGGER_TIME) {
        _triggered = false;
        _triggerTime = loopTime;
        
        // disable trigger
        digitalWrite(_params[WATER_SENSOR_PARAM_PINS_E].data.uint8[0], LOW);

        // read sensor value
        uint32_t raw = analogRead(_params[WATER_SENSOR_PARAM_PINS_E].data.uint8[1]);

        // publish new values
        updateAndPublishParam(&_params[WATER_SENSOR_PARAM_RAW_E], (uint8_t*)&raw, sizeof(raw));

        // see if raw exceeds threshold
        float alarm = (raw >= _params[WATER_SENSOR_PARAM_THRESHOLD_E].data.uint32[0]) ? 1 : 0;

        updateAndPublishParam(&_params[WATER_SENSOR_PARAM_ALARM_E], (uint8_t*)&alarm, sizeof(alarm));
    }

  } else {
    // wait for interval
    if (loopTime > _triggerTime + WATER_SENSOR_SAMPLE_INTERVAL) {
        _triggered = true;
        _triggerTime = loopTime;
        // trigger sensor
        digitalWrite(_params[WATER_SENSOR_PARAM_PINS_E].data.uint8[0], HIGH);
    }

  }
}
