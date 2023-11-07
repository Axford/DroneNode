#include "WindModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include <SPIFFS.h>
#include "DroneSystem.h"
#include "../navMath.h"

// @type Wind

unsigned long _globalWindCounter;

WindModule::WindModule(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
 {
   setTypeName(FPSTR(WIND_STR_WIND));
   //_params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = WIND_I2C_ADDRESS;
   _sensor = NULL;

  _dirSample =0;

   _globalWindCounter = 0;
   _speedSample = 0;
   _lastSpeedSampleTime = 0;

   for (uint8_t i=0; i<WIND_SPEED_SAMPLES; i++) {
     _speedSamples[i] = 0;
   }

   // subs
   initSubs(WIND_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[WIND_SUB_HEADING_E];
   sub->addrParam = WIND_SUB_HEADING_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WIND_SUB_HEADING);
   setParamName(FPSTR(STRING_HEADING), &sub->param);


   // pubs
   initParams(WIND_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = WIND_I2C_ADDRESS;

   // init param entries
   _params[WIND_PARAM_DIRECTION_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, WIND_PARAM_DIRECTION);
   _params[WIND_PARAM_DIRECTION_E].name = FPSTR(STRING_DIRECTION);
   _params[WIND_PARAM_DIRECTION_E].nameLen = sizeof(STRING_DIRECTION);
   _params[WIND_PARAM_DIRECTION_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[WIND_PARAM_SPEED_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, WIND_PARAM_SPEED);
   _params[WIND_PARAM_SPEED_E].name = FPSTR(STRING_SPEED);
   _params[WIND_PARAM_SPEED_E].nameLen = sizeof(STRING_SPEED);
   _params[WIND_PARAM_SPEED_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[WIND_PARAM_PINS_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WIND_PARAM_PINS);
   _params[WIND_PARAM_PINS_E].name = FPSTR(STRING_PINS);
   _params[WIND_PARAM_PINS_E].nameLen = sizeof(STRING_PINS);
   _params[WIND_PARAM_PINS_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);

   _params[WIND_PARAM_CENTRE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WIND_PARAM_CENTRE);
   _params[WIND_PARAM_CENTRE_E].name = FPSTR(STRING_CENTRE);
   _params[WIND_PARAM_CENTRE_E].nameLen = sizeof(STRING_CENTRE);
   _params[WIND_PARAM_CENTRE_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[WIND_PARAM_CENTRE_E].data.f[0] = 0;

   _params[WIND_PARAM_WIND_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, WIND_PARAM_WIND);
   _params[WIND_PARAM_WIND_E].name = FPSTR(STRING_WIND);
   _params[WIND_PARAM_WIND_E].nameLen = sizeof(STRING_WIND);
   _params[WIND_PARAM_WIND_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[WIND_PARAM_MODE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WIND_PARAM_MODE);
   _params[WIND_PARAM_MODE_E].name = FPSTR(STRING_MODE);
   _params[WIND_PARAM_MODE_E].nameLen = sizeof(STRING_MODE);
   _params[WIND_PARAM_MODE_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[WIND_PARAM_MODE_E].data.uint8[0] = WIND_MODE_STANDARD;

   _params[WIND_PARAM_SAMPLES_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WIND_PARAM_SAMPLES);
   _params[WIND_PARAM_SAMPLES_E].name = FPSTR(STRING_SAMPLES);
   _params[WIND_PARAM_SAMPLES_E].nameLen = sizeof(STRING_SAMPLES);
   _params[WIND_PARAM_SAMPLES_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   // @default samples=1
   _params[WIND_PARAM_SAMPLES_E].data.uint8[0] = 1;

   _params[WIND_PARAM_STATS_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, WIND_PARAM_STATS);
   _params[WIND_PARAM_STATS_E].name = FPSTR(STRING_STATS);
   _params[WIND_PARAM_STATS_E].nameLen = sizeof(STRING_STATS);
   _params[WIND_PARAM_STATS_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[WIND_PARAM_STATS_E].data.f[0] = 0;

}

WindModule::~WindModule() {
  if (_sensor) delete _sensor;
}


void WindModule::doReset() {
  I2CBaseModule::doReset();
}

void IRAM_ATTR WindModule::ISR() {
  _globalWindCounter++;
}


void WindModule::setup() {
  I2CBaseModule::setup();

  if (!_sensor) {
    Log.noticeln("[Wind.s]");

    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
    _sensor = new AMS_5600();

    if (!I2CBaseModule::isAlive()) {
      Log.errorln("[Wind.s] No sensor found");
      setError(1);
      disable();
      return;
    }

    // debug output

    Log.noticeln("[Wind.s] Raw Angle: %d", _sensor->getRawAngle() );
    Log.noticeln("[Wind.s] detectMagnet: %d", _sensor->detectMagnet() );
    Log.noticeln("[Wind.s] getMagnetStrength: %d", _sensor->getMagnetStrength() );
    Log.noticeln("[Wind.s] getAgc: %d", _sensor->getAgc() );
    Log.noticeln("[Wind.s] getMagnitude: %d", _sensor->getMagnitude() );
  }

  if (_params[WIND_PARAM_PINS_E].data.uint8[0] > 0) {
    if (_ds->requestPin(_params[WIND_PARAM_PINS_E].data.uint8[0], DRONE_SYSTEM_PIN_CAP_INPUT, this)) {
      pinMode(_params[WIND_PARAM_PINS_E].data.uint8[0], INPUT_PULLUP);

      // attach interrupt
      attachInterrupt( _params[WIND_PARAM_PINS_E].data.uint8[0], ISR, FALLING );

    } else {
      Log.errorln(F("[W.s] Speed pin unavailable %u"), _id);
    }

  } else {
    Log.errorln(F("[W.s] Undefined speed pin %u"), _id);
  }
}


void WindModule::loop() {
  I2CBaseModule::loop();

  unsigned long loopTime = millis();
  float dt = (loopTime - _lastSpeedSampleTime) / 1000.0;
  _lastSpeedSampleTime = loopTime;

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  // update local Wind direction
  // ---------------------------------

  float ang = 360 * _sensor->getRawAngle() / 4095;

  if (_params[WIND_PARAM_MODE_E].data.uint8[0] == WIND_MODE_INVERTED) {
    ang = -ang;
  }

  ang -= _params[WIND_PARAM_CENTRE_E].data.f[0];

  ang = fmod(ang, 360);
  if (ang < 0) ang += 360;

  updateAndPublishParam(&_params[WIND_PARAM_DIRECTION_E], (uint8_t*)&ang, sizeof(ang));


  // update moving average for Speed
  // -----------------------------------
  
  _speedSamples[_speedSample] = _globalWindCounter / dt;
  _globalWindCounter = 0;

  _speedSample++;
  if (_speedSample >= WIND_SPEED_SAMPLES) _speedSample = 0;

  // add up all the windcount rates to get the average
  float speed = 0;
  for (uint8_t i=0; i<WIND_SPEED_SAMPLES; i++) {
    speed += _speedSamples[i];
  }
  speed = speed / WIND_SPEED_SAMPLES;

  // convert to actual speed, based on 1.25m per revolution (count)
  speed = speed * 1.25;

  // then convert from m/s to knots
  speed = speed * 1.94384;

  updateAndPublishParam(&_params[WIND_PARAM_SPEED_E], (uint8_t*)&speed, sizeof(speed));


  // update moving average for World Direction
  // -------------------------------------------

  float w = _subs[WIND_SUB_HEADING_E].param.data.f[0] + ang;
  w = fmod(w, 360);
  if (w < 0) w += 360;

  // update sample count
  if (_dirSample < _params[WIND_PARAM_SAMPLES_E].data.uint8[0]) _dirSample++;
  if (_dirSample < 1) _dirSample = 1;

  // recalculate w as shortest circular distance from current average... normal moving average won't work otherwise!
  float w1 = shortestSignedDistanceBetweenCircularValues(_params[WIND_PARAM_WIND_E].data.f[0], w);
  // then add back to current average to get as a angle we can average
  w1 = _params[WIND_PARAM_WIND_E].data.f[0] + w1;

  float variance = abs(w1);

  // calc moving average variance
  variance = ((_params[WIND_PARAM_STATS_E].data.f[0] * (_dirSample-1)) + variance) / _dirSample;

  // update moving average
  w = ((_params[WIND_PARAM_WIND_E].data.f[0] * (_dirSample-1)) + w1) / _dirSample;
  
  w = fmod(w, 360);
  if (w < 0) w += 360;

  updateAndPublishParam(&_params[WIND_PARAM_WIND_E], (uint8_t*)&w, sizeof(w));

  updateAndPublishParam(&_params[WIND_PARAM_STATS_E], (uint8_t*)&variance, sizeof(variance));
}
