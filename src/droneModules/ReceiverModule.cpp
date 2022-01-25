#include "ReceiverModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

// globals for use in ISRs
uint8_t _globalReceiverPins[2];
unsigned long _globalReceiverRawTimers[2];

ReceiverModule::ReceiverModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(RECEIVER_STR_RECEIVER));

   // subs
   initSubs(RECEIVER_SUBS);


   // pubs
   initParams(RECEIVER_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[RECEIVER_PARAM_PINS_E];
   param->param = RECEIVER_PARAM_PINS;
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[RECEIVER_PARAM_PINS_E].data.uint8[0] = 0;


   param = &_params[RECEIVER_PARAM_VALUE1_E];
   param->param = RECEIVER_PARAM_VALUE1;
   setParamName(FPSTR(STRING_VALUE1), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[RECEIVER_PARAM_VALUE2_E];
   param->param = RECEIVER_PARAM_VALUE2;
   setParamName(FPSTR(STRING_VALUE2), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

}


DEM_NAMESPACE* ReceiverModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(RECEIVER_STR_RECEIVER,0,true);
}

void ReceiverModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_PINS, DRONE_LINK_MSG_TYPE_UINT8_T, ph);

}


void IRAM_ATTR ReceiverModule::ISR1() {
  static unsigned long _startTime = 0;

  if (digitalRead(_globalReceiverPins[0])) {
    // rising
    _startTime = micros();
  } else {
    // falling
    _globalReceiverRawTimers[0] = micros() - _startTime;
  }
}


void IRAM_ATTR ReceiverModule::ISR2() {
  static unsigned long _startTime = 0;

  if (digitalRead(_globalReceiverPins[1])) {
    // rising
    _startTime = micros();
  } else {
    // falling
    _globalReceiverRawTimers[1] = micros() - _startTime;
  }
}


void ReceiverModule::setup() {
  DroneModule::setup();

  if (_params[RECEIVER_PARAM_PINS_E].data.uint8[0] > 0) {
    _globalReceiverPins[0] = _params[RECEIVER_PARAM_PINS_E].data.uint8[0];
    _globalReceiverRawTimers[0] = 0;
    attachInterrupt(_params[RECEIVER_PARAM_PINS_E].data.uint8[0], ISR1, CHANGE);
  } else {
    Log.errorln(F("Undefined pin 0 %d"), _params[RECEIVER_PARAM_PINS_E].data.uint8[0]);
  }

  if (_params[RECEIVER_PARAM_PINS_E].data.uint8[1] > 0) {
    _globalReceiverPins[1] = _params[RECEIVER_PARAM_PINS_E].data.uint8[1];
    _globalReceiverRawTimers[1] = 0;
    attachInterrupt(_params[RECEIVER_PARAM_PINS_E].data.uint8[1], ISR2, CHANGE);

  } else {
    Log.errorln(F("Undefined pin 1 %d"), _params[RECEIVER_PARAM_PINS_E].data.uint8[1]);
  }
}

void ReceiverModule::update() {
  if (_error > 0 || !_setupDone) return;

}

void ReceiverModule::loop() {
  DroneModule::loop();

  // raw values
  updateAndPublishParam(&_params[RECEIVER_PARAM_OUTPUT_E], (uint8_t*)&_globalReceiverRawTimers, sizeof(_globalReceiverRawTimers));


  // calculate and publish new output values (in range -1..1)
  float v;

  // channel 1
  if (_globalReceiverRawTimers[0] > 500 && _globalReceiverRawTimers[0] < 2400) {
    v = 2 * (_globalReceiverRawTimers[0] - 500) / (2400 - 500) - 1;
  } else {
    v = 0;
  }
  updateAndPublishParam(&_params[RECEIVER_PARAM_VALUE1_E], (uint8_t*)&v, sizeof(v));


  // channel 2
  if (_globalReceiverRawTimers[1] > 500 && _globalReceiverRawTimers[1] < 2400) {
    v = 2 * (_globalReceiverRawTimers[1] - 500) / (2400 - 500) - 1;
  } else {
    v = 0;
  }
  updateAndPublishParam(&_params[RECEIVER_PARAM_VALUE2_E], (uint8_t*)&v, sizeof(v));


}
