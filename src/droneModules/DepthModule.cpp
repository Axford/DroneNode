#include "DepthModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

DepthModule::DepthModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(DEPTH_STR_DEPTH));

   // set default interval to 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   // subs
   initSubs(DEPTH_SUBS);

/*   DRONE_PARAM_SUB *sub;

   sub = &_subs[DEPTH_SUB_SPEED_E];
   sub->addrParam = DEPTH_SUB_SPEED_ADDR;
   sub->param.param = DEPTH_SUB_SPEED;
   setParamName(FPSTR(STRING_SPEED), &sub->param);
*/

   // pubs
   initParams(DEPTH_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[DEPTH_PARAM_PINS_E];
   param->param = DEPTH_PARAM_PINS;
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 2);
   _params[DEPTH_PARAM_PINS_E].data.uint8[0] = 0;

   param = &_params[DEPTH_PARAM_SPEED_E];
   param->param = DEPTH_PARAM_SPEED;
   setParamName(FPSTR(STRING_PWM_CHANNEL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[DEPTH_PARAM_SPEED_E].data.f[0] = 1480;

   param = &_params[DEPTH_PARAM_LIMITS_E];
   param->param = DEPTH_PARAM_LIMITS;
   setParamName(FPSTR(STRING_LIMITS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   _params[DEPTH_PARAM_LIMITS_E].data.f[0] = 0.25;
   _params[DEPTH_PARAM_LIMITS_E].data.f[1] = 10;

   param = &_params[DEPTH_PARAM_DEPTH_E];
   param->param = DEPTH_PARAM_DEPTH;
   setParamName(FPSTR(STRING_DEPTH), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[DEPTH_PARAM_DEPTH_E].data.f[0] = 0;

}

DEM_NAMESPACE* DepthModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(DEPTH_STR_DEPTH,0,true);
}

void DepthModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_PINS, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_SPEED, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_LIMITS, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}


void DepthModule::setup() {
  DroneModule::setup();

  if (_params[DEPTH_PARAM_PINS_E].data.uint8[0] > 0) {

    // set trigger to output
    pinMode(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_TRIGGER], OUTPUT);
    digitalWrite(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_TRIGGER], LOW);

    // set echo to input
    pinMode(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_ECHO], INPUT);

  } else {
    Log.errorln(F("[DM.s] Undefined pins %u"), _id);
    setError(1);
    disable();
  }
}


void DepthModule::loop() {
  DroneModule::loop();

  // calc timeout value in microseconds
  // 2 * 1000000 * max measurable value / speed of sound
  unsigned long timeout = 2000000 * _params[DEPTH_PARAM_LIMITS_E].data.f[1] / _params[DEPTH_PARAM_SPEED_E].data.f[0];

  // send a pulse
  digitalWrite(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_TRIGGER], HIGH);
  delayMicroseconds(10);
  digitalWrite(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_TRIGGER], LOW);

  // wait for return echo and calcuate duration
  const unsigned long duration= pulseIn(_params[DEPTH_PARAM_PINS_E].data.uint8[DEPTH_PIN_ECHO], HIGH, timeout);

  // convert to distance (in meters)
  // Need to half the duration as it represents twice the depth
  float d = _params[DEPTH_PARAM_SPEED_E].data.f[0] * duration / 2000000;

  // publish new depth value
  updateAndPublishParam(&_params[DEPTH_PARAM_DEPTH_E], (uint8_t*)&d, sizeof(d));

  if(duration==0){
    Serial.println("Warning: no pulse from sensor");
  } else {
    Serial.print("Depth:");
    Serial.println(d);
  }
}
