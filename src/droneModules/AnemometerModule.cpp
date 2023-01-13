#include "AnemometerModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "DroneSystem.h"
#include "../navMath.h"

unsigned long _globalAnemometerCounter2;

AnemometerModule::AnemometerModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(ANEMOMETER_STR_ANEMOMETER));

   _globalAnemometerCounter2 = 0;
   _speedSample = 0;
   _lastSpeedSampleTime = 0;

   for (uint8_t i=0; i<ANEMOMETER_SPEED_SAMPLES; i++) {
     _speedSamples[i] = 0;
   }

   // subs
   initSubs(ANEMOMETER_SUBS);

   //DRONE_PARAM_SUB *sub;


   // pubs
   initParams(ANEMOMETER_PARAM_ENTRIES);

   // init param entries
   _params[ANEMOMETER_PARAM_SPEED_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, ANEMOMETER_PARAM_SPEED);
   _params[ANEMOMETER_PARAM_SPEED_E].name = FPSTR(STRING_SPEED);
   _params[ANEMOMETER_PARAM_SPEED_E].nameLen = sizeof(STRING_SPEED);
   _params[ANEMOMETER_PARAM_SPEED_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[ANEMOMETER_PARAM_PINS_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ANEMOMETER_PARAM_PINS);
   _params[ANEMOMETER_PARAM_PINS_E].name = FPSTR(STRING_PINS);
   _params[ANEMOMETER_PARAM_PINS_E].nameLen = sizeof(STRING_PINS);
   _params[ANEMOMETER_PARAM_PINS_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);


}



DEM_NAMESPACE* AnemometerModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(ANEMOMETER_STR_ANEMOMETER,0,true);
}

void AnemometerModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {

}


void IRAM_ATTR AnemometerModule::ISR() {
  _globalAnemometerCounter2++;
}


void AnemometerModule::setup() {
  DroneModule::setup();

  if (_params[ANEMOMETER_PARAM_PINS_E].data.uint8[0] > 0) {
    if (_ds->requestPin(_params[ANEMOMETER_PARAM_PINS_E].data.uint8[0], DRONE_SYSTEM_PIN_CAP_INPUT, this)) {
      pinMode(_params[ANEMOMETER_PARAM_PINS_E].data.uint8[0], INPUT_PULLUP);

      // attach interrupt
      attachInterrupt( _params[ANEMOMETER_PARAM_PINS_E].data.uint8[0], ISR, FALLING );

    } else {
      Log.errorln(F("[W.s] Speed pin unavailable %u"), _id);
    }

  } else {
    Log.errorln(F("[W.s] Undefined speed pin %u"), _id);
  }
}


void AnemometerModule::loop() {
  DroneModule::loop();

  unsigned long loopTime = millis();
  float dt = (loopTime - _lastSpeedSampleTime) / 1000.0;
  _lastSpeedSampleTime = loopTime;

  // update moving average for Speed
  // -----------------------------------
  
  _speedSamples[_speedSample] = _globalAnemometerCounter2 / dt;
  _globalAnemometerCounter2 = 0;

  _speedSample++;
  if (_speedSample >= ANEMOMETER_SPEED_SAMPLES) _speedSample = 0;

  // add up all the windcount rates to get the average
  float speed = 0;
  for (uint8_t i=0; i<ANEMOMETER_SPEED_SAMPLES; i++) {
    speed += _speedSamples[i];
  }
  speed = speed / ANEMOMETER_SPEED_SAMPLES;

  // convert to actual speed, based on 1.25m per revolution (count)
  speed = speed * 1.25;

  // then convert from m/s to knots
  speed = speed * 1.94384;

  updateAndPublishParam(&_params[ANEMOMETER_PARAM_SPEED_E], (uint8_t*)&speed, sizeof(speed));

}
