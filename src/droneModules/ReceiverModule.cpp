#include "ReceiverModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

// globals for use in ISRs
uint8_t _globalReceiveMode = RECEIVER_MODE_PPM;
uint8_t _globalReceiverPins[4];
unsigned long _globalReceiverRawTimers[4];
unsigned long _globalLastReceiverSignal;

ReceiverModule::ReceiverModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(RECEIVER_STR_RECEIVER));

   _globalLastReceiverSignal = 0;
   _globalReceiverPins[0] = 0;
   _globalReceiverPins[1] = 0;
   _globalReceiverRawTimers[0] = 0;
   _globalReceiverRawTimers[1] = 0;

   // subs
   initSubs(RECEIVER_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[RECEIVER_SUB_INPUT1_E];
   sub->addrParam = RECEIVER_SUB_INPUT1_ADDR;
   sub->param.param = RECEIVER_SUB_INPUT1;
   setParamName(FPSTR(STRING_INPUT1), &sub->param);

   sub = &_subs[RECEIVER_SUB_INPUT2_E];
   sub->addrParam = RECEIVER_SUB_INPUT2_ADDR;
   sub->param.param = RECEIVER_SUB_INPUT2;
   setParamName(FPSTR(STRING_INPUT2), &sub->param);

   sub = &_subs[RECEIVER_SUB_INPUT3_E];
   sub->addrParam = RECEIVER_SUB_INPUT3_ADDR;
   sub->param.param = RECEIVER_SUB_INPUT3;
   setParamName(FPSTR(STRING_INPUT3), &sub->param);

   sub = &_subs[RECEIVER_SUB_INPUT4_E];
   sub->addrParam = RECEIVER_SUB_INPUT4_ADDR;
   sub->param.param = RECEIVER_SUB_INPUT4;
   setParamName(FPSTR(STRING_INPUT4), &sub->param);

   sub = &_subs[RECEIVER_SUB_SWITCH_E];
   sub->addrParam = RECEIVER_SUB_SWITCH_ADDR;
   sub->param.param = RECEIVER_SUB_SWITCH;
   setParamName(FPSTR(STRING_SWITCH), &sub->param);
   sub->param.data.f[0] = 1; // default to active mode


   // pubs
   initParams(RECEIVER_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[RECEIVER_PARAM_PINS_E];
   param->param = RECEIVER_PARAM_PINS;
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 4);
   _params[RECEIVER_PARAM_PINS_E].data.uint8[0] = 0;
   _params[RECEIVER_PARAM_PINS_E].data.uint8[1] = 0;
   _params[RECEIVER_PARAM_PINS_E].data.uint8[2] = 0;
   _params[RECEIVER_PARAM_PINS_E].data.uint8[3] = 0;


   param = &_params[RECEIVER_PARAM_VALUE1_E];
   param->param = RECEIVER_PARAM_VALUE1;
   setParamName(FPSTR(STRING_VALUE1), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[RECEIVER_PARAM_VALUE2_E];
   param->param = RECEIVER_PARAM_VALUE2;
   setParamName(FPSTR(STRING_VALUE2), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[RECEIVER_PARAM_VALUE3_E];
   param->param = RECEIVER_PARAM_VALUE3;
   setParamName(FPSTR(STRING_VALUE3), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[RECEIVER_PARAM_VALUE4_E];
   param->param = RECEIVER_PARAM_VALUE4;
   setParamName(FPSTR(STRING_VALUE4), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[RECEIVER_PARAM_LIMITS_E];
   param->param = RECEIVER_PARAM_LIMITS;
   setParamName(FPSTR(STRING_LIMITS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT32_T, 8);
   param->data.uint32[0] = 1000;
   param->data.uint32[1] = 2000;

   param = &_params[RECEIVER_PARAM_INPUT_E];
   param->param = RECEIVER_PARAM_INPUT;
   setParamName(FPSTR(STRING_INPUT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 16);

   param = &_params[RECEIVER_PARAM_MODE_E];
   param->param = RECEIVER_PARAM_MODE;
   setParamName(FPSTR(STRING_MODE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   param->data.uint8[0] = RECEIVER_MODE_PPM;
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
  dem->registerCommand(ns, STRING_LIMITS, DRONE_LINK_MSG_TYPE_UINT32_T, ph);
  dem->registerCommand(ns, STRING_MODE, DRONE_LINK_MSG_TYPE_UINT8_T, ph);

  dem->registerCommand(ns, STRING_INPUT1, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$input1"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_INPUT2, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$input2"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_INPUT3, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$input3"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_INPUT4, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$input4"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_SWITCH, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$switch"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
}


void IRAM_ATTR ReceiverModule::ISR1() {
  static unsigned long _startTime = 0;

  static unsigned long microsAtLastPulse = 0;
  static uint8_t pulseCounter = 0;


  if (_globalReceiveMode == RECEIVER_MODE_PPM) {
    // -----------------------------------------------------------
    // PPM MODE
    // -----------------------------------------------------------

    unsigned long previousMicros = microsAtLastPulse;
    microsAtLastPulse = micros();
    unsigned long pulseTime = microsAtLastPulse - previousMicros;

    if (pulseTime > RECEIVER_PPM_BLANK_TIME) {
      // end of frame (and start of a new one)
      pulseCounter = 0;
      _globalLastReceiverSignal = millis();

    } else {
      // new pulse
      if (pulseCounter < 4) {
        _globalReceiverRawTimers[pulseCounter] = pulseTime;
      }
      pulseCounter++;
    }


  } else {
    // -----------------------------------------------------------
    // PWM MODE
    // -----------------------------------------------------------
    if (digitalRead(_globalReceiverPins[0])) {
      // rising
      _startTime = micros();
    } else {
      // falling
      _globalReceiverRawTimers[0] = micros() - _startTime;
      _globalLastReceiverSignal = millis();
    }
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
    _globalLastReceiverSignal = millis();
  }
}

void IRAM_ATTR ReceiverModule::ISR3() {
  static unsigned long _startTime = 0;

  if (digitalRead(_globalReceiverPins[2])) {
    // rising
    _startTime = micros();
  } else {
    // falling
    _globalReceiverRawTimers[2] = micros() - _startTime;
    _globalLastReceiverSignal = millis();
  }
}

void IRAM_ATTR ReceiverModule::ISR4() {
  static unsigned long _startTime = 0;

  if (digitalRead(_globalReceiverPins[3])) {
    // rising
    _startTime = micros();
  } else {
    // falling
    _globalReceiverRawTimers[3] = micros() - _startTime;
    _globalLastReceiverSignal = millis();
  }
}


void ReceiverModule::setup() {
  DroneModule::setup();

  _globalReceiveMode = _params[RECEIVER_PARAM_MODE_E].data.uint8[0];

  if (_params[RECEIVER_PARAM_MODE_E].data.uint8[0] == RECEIVER_MODE_PPM) {

    if (_params[RECEIVER_PARAM_PINS_E].data.uint8[0] > 0) {
      _globalReceiverPins[0] = _params[RECEIVER_PARAM_PINS_E].data.uint8[0];
      _globalReceiverRawTimers[0] = 0;
      pinMode(_params[RECEIVER_PARAM_PINS_E].data.uint8[0], INPUT);
      attachInterrupt(_params[RECEIVER_PARAM_PINS_E].data.uint8[0], ISR1, RISING);
    }


  } else {

    if (_params[RECEIVER_PARAM_PINS_E].data.uint8[0] > 0) {
      _globalReceiverPins[0] = _params[RECEIVER_PARAM_PINS_E].data.uint8[0];
      _globalReceiverRawTimers[0] = 0;
      pinMode(_params[RECEIVER_PARAM_PINS_E].data.uint8[0], INPUT);
      attachInterrupt(_params[RECEIVER_PARAM_PINS_E].data.uint8[0], ISR1, CHANGE);
    } else {
      //Log.errorln(F("Undefined pin 0 %d"), _params[RECEIVER_PARAM_PINS_E].data.uint8[0]);
    }

    if (_params[RECEIVER_PARAM_PINS_E].data.uint8[1] > 0) {
      _globalReceiverPins[1] = _params[RECEIVER_PARAM_PINS_E].data.uint8[1];
      _globalReceiverRawTimers[1] = 0;
      pinMode(_params[RECEIVER_PARAM_PINS_E].data.uint8[1], INPUT);
      attachInterrupt(_params[RECEIVER_PARAM_PINS_E].data.uint8[1], ISR2, CHANGE);

    } else {
      //Log.errorln(F("Undefined pin 1 %d"), _params[RECEIVER_PARAM_PINS_E].data.uint8[1]);
    }

    if (_params[RECEIVER_PARAM_PINS_E].data.uint8[2] > 0) {
      _globalReceiverPins[2] = _params[RECEIVER_PARAM_PINS_E].data.uint8[2];
      _globalReceiverRawTimers[2] = 0;
      pinMode(_params[RECEIVER_PARAM_PINS_E].data.uint8[2], INPUT);
      attachInterrupt(_params[RECEIVER_PARAM_PINS_E].data.uint8[2], ISR3, CHANGE);

    }

    if (_params[RECEIVER_PARAM_PINS_E].data.uint8[3] > 0) {
      _globalReceiverPins[3] = _params[RECEIVER_PARAM_PINS_E].data.uint8[3];
      _globalReceiverRawTimers[3] = 0;
      pinMode(_params[RECEIVER_PARAM_PINS_E].data.uint8[3], INPUT);
      attachInterrupt(_params[RECEIVER_PARAM_PINS_E].data.uint8[3], ISR4, CHANGE);

    }

  }
}

void ReceiverModule::update() {
  if (_error > 0 || !_setupDone) return;

}

float ReceiverModule::rawToValue(uint8_t chan) {
  float v;
  float minV = _params[RECEIVER_PARAM_LIMITS_E].data.uint32[0];
  float maxV = _params[RECEIVER_PARAM_LIMITS_E].data.uint32[1];
  if (_globalReceiverRawTimers[chan] > minV && _globalReceiverRawTimers[chan] < maxV) {
    v = (2 * (_globalReceiverRawTimers[chan] - minV) / (maxV - minV)) - 1;
  } else {
    v = 0;
  }
  return v;
}

void ReceiverModule::loop() {
  DroneModule::loop();

  boolean validSignal = (millis() - _globalLastReceiverSignal) < 5000;

  // raw values
  updateAndPublishParam(&_params[RECEIVER_PARAM_INPUT_E], (uint8_t*)&_globalReceiverRawTimers, sizeof(_globalReceiverRawTimers));

  boolean passthroughMode = _subs[RECEIVER_SUB_SWITCH_E].param.data.f[0] < 0.5;

  float v = 0;



  // calculate and publish new output values (in range -1..1)

  // channel 1
  if (passthroughMode && _subs[RECEIVER_SUB_INPUT1_E].addr.channel > 0) {
    v = _subs[RECEIVER_SUB_INPUT1_E].param.data.f[0];
  } else {
    v = validSignal ? rawToValue(0) : 0;
  }
  updateAndPublishParam(&_params[RECEIVER_PARAM_VALUE1_E], (uint8_t*)&v, sizeof(v));

  // channel 2
  if (passthroughMode && _subs[RECEIVER_SUB_INPUT2_E].addr.channel > 0) {
    v = _subs[RECEIVER_SUB_INPUT2_E].param.data.f[0];
  } else {
    v = validSignal ? rawToValue(1) : 0;
  }
  updateAndPublishParam(&_params[RECEIVER_PARAM_VALUE2_E], (uint8_t*)&v, sizeof(v));

  // channel 3
  if (passthroughMode && _subs[RECEIVER_SUB_INPUT3_E].addr.channel > 0) {
    v = _subs[RECEIVER_SUB_INPUT3_E].param.data.f[0];
  } else {
    v = validSignal ? rawToValue(2) : 0;
  }
  updateAndPublishParam(&_params[RECEIVER_PARAM_VALUE3_E], (uint8_t*)&v, sizeof(v));

  // channel 4
  if (passthroughMode && _subs[RECEIVER_SUB_INPUT4_E].addr.channel > 0) {
    v = _subs[RECEIVER_SUB_INPUT4_E].param.data.f[0];
  } else {
    v = validSignal ? rawToValue(3) : 0;
  }
  updateAndPublishParam(&_params[RECEIVER_PARAM_VALUE4_E], (uint8_t*)&v, sizeof(v));



}
