#include "PolarModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"

// @type Polar

PolarModule::PolarModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(POLAR_STR_POLAR));

   // init polarVals
   for (uint8_t i=0; i<16; i++) _polarVals[i] = 0;
   _startTime = 0;
   _startPos[0] = 0;
   _startPos[1] = 0;

   _region = POLAR_REGION_OUT;

   // set default interval to 1000
   // @default interval = 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   // subs
   initSubs(POLAR_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[POLAR_SUB_LOCATION_E];
   sub->addrParam = POLAR_SUB_LOCATION_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, POLAR_SUB_LOCATION);
   sub->param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   setParamName(FPSTR(STRING_LOCATION), &sub->param);
   sub->param.data.f[0] =0;
   sub->param.data.f[1] =0;

   sub = &_subs[POLAR_SUB_SOG_E];
   sub->addrParam = POLAR_SUB_SOG_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, POLAR_SUB_SOG);
   setParamName(FPSTR(STRING_SOG), &sub->param);

   sub = &_subs[POLAR_SUB_WIND_E];
   sub->addrParam = POLAR_SUB_WIND_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, POLAR_SUB_WIND);
   setParamName(FPSTR(STRING_WIND), &sub->param);

   sub = &_subs[POLAR_SUB_WIND_SPEED_E];
   sub->addrParam = POLAR_SUB_WIND_SPEED_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, POLAR_SUB_WIND_SPEED);
   setParamName(FPSTR(STRING_WIND_SPEED), &sub->param);

   sub = &_subs[POLAR_SUB_HEADING_E];
   sub->addrParam = POLAR_SUB_HEADING_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, POLAR_SUB_HEADING);
   setParamName(FPSTR(STRING_HEADING), &sub->param);


   // pubs
   initParams(POLAR_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[POLAR_PARAM_TARGET_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, POLAR_PARAM_TARGET);
   setParamName(FPSTR(STRING_TARGET), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   param->data.f[0] = 0;
   param->data.f[1] = 0;

   param = &_params[POLAR_PARAM_MODE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, POLAR_PARAM_MODE);
   setParamName(FPSTR(STRING_MODE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   param->data.uint8[0] = POLAR_MODE_PASSTHROUGH;

   param = &_params[POLAR_PARAM_THRESHOLD_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, POLAR_PARAM_THRESHOLD);
   setParamName(FPSTR(STRING_THRESHOLD), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   // @default threshold=1,10
   param->data.f[0] = 1;  // min SOG
   param->data.f[1] = 10;  // acceptable heading deviation

   param = &_params[POLAR_PARAM_POLAR_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, POLAR_PARAM_POLAR);
   setParamName(FPSTR(STRING_POLAR), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);
   for (uint8_t i=0; i<16; i++) param->data.uint8[i] = 0;

   param = &_params[POLAR_PARAM_SAMPLES_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, POLAR_PARAM_SAMPLES);
   setParamName(FPSTR(STRING_SAMPLES), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);
   for (uint8_t i=0; i<16; i++) param->data.uint8[i] = 0;

   param = &_params[POLAR_PARAM_RADIUS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, POLAR_PARAM_RADIUS);
   setParamName(FPSTR(STRING_RADIUS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[0] = 0; // inner
   param->data.f[1] = 0; // mid
   param->data.f[2] = 0; // outer radii

   param = &_params[POLAR_PARAM_ADJ_HEADING_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, POLAR_PARAM_ADJ_HEADING);
   setParamName(FPSTR(STRING_ADJ_HEADING), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);

}


uint8_t PolarModule::polarIndexForAngle(float ang) {
  //float w = _subs[SAILOR_SUB_WIND_E].param.data.f[0];

  //float polarAng = fmod(ang - w, 360);
  float polarAng = fmod(ang, 360);
  if (polarAng < 0) polarAng += 360;
  uint8_t polarIndex = polarAng / 11.25;
  if (polarIndex > 31) polarIndex -= 32;
  if (polarIndex > 15) polarIndex = 31 - polarIndex;

  return polarIndex;
}


void PolarModule::updatePolar() {
  // recalculate from sampled values

  // calc averages per bin and find peak value
  float avgs[16];
  float peakVal = 0;
  Serial.print("pv: ");
  for (uint8_t i=0; i<16; i++) {
    Serial.print(_polarVals[i]);
    Serial.print(" ");
    if ( _params[POLAR_PARAM_SAMPLES_E].data.uint8[i] > 0 ) {
      avgs[i] = _polarVals[i] / _params[POLAR_PARAM_SAMPLES_E].data.uint8[i];
    } else {
      avgs[i] = 0;
    }

    if (avgs[i] > peakVal) peakVal = avgs[i];
  }
  Serial.println("");

  // update polar
  if (peakVal  > 0) {
    for (uint8_t i=0; i<16; i++) {
      _params[POLAR_PARAM_POLAR_E].data.uint8[i] = (255 * avgs[i] / peakVal);
    }
    publishParamEntry(&_params[POLAR_PARAM_POLAR_E]);
  }
}


void PolarModule::setup() {
  DroneModule::setup();

}


void PolarModule::update() {
  if (!_setupDone) return;

  if (_params[POLAR_PARAM_MODE_E].data.uint8[0] == POLAR_MODE_PASSTHROUGH) {

    // pass heading to adjHeading
    float tempF = _subs[POLAR_SUB_HEADING_E].param.data.f[0];
    updateAndPublishParam(&_params[POLAR_PARAM_ADJ_HEADING_E], (uint8_t*)&tempF, sizeof(tempF));
  }
}


void PolarModule::loop() {
  DroneModule::loop();

  switch (_params[POLAR_PARAM_MODE_E].data.uint8[0]) {
    case POLAR_MODE_PASSTHROUGH: break;
    case POLAR_MODE_ACTIVE: loopActive(); break;
    case POLAR_MODE_RESET: loopReset(); break;
  }
}


void PolarModule::loopActive() {
  // see how close we are to our target
  float dist = calculateDistanceBetweenCoordinates(
    _subs[POLAR_SUB_LOCATION_E].param.data.f[0],
    _subs[POLAR_SUB_LOCATION_E].param.data.f[1],
    _params[POLAR_PARAM_TARGET_E].data.f[0],
    _params[POLAR_PARAM_TARGET_E].data.f[1]
  );

  uint8_t newRegion = POLAR_REGION_OUT;
  if (dist < _params[POLAR_PARAM_RADIUS_E].data.f[0]) {
    newRegion = POLAR_REGION_IN;
  } else if (dist < _params[POLAR_PARAM_RADIUS_E].data.f[2]) {
    newRegion = POLAR_REGION_MID;
  }

  // see if we drifted outside our target area
  if (newRegion != _region && newRegion == POLAR_REGION_OUT) {
    //Serial.println("[PM.lA] out");
    // switch to passthrough
    
    uint8_t newMode = POLAR_MODE_PASSTHROUGH;
    updateAndPublishParam(&_params[POLAR_PARAM_MODE_E], (uint8_t*)&newMode, sizeof(newMode));

    _region = newRegion;
    return;
  }

  // if in the mid region...
  if (newRegion == POLAR_REGION_MID || newRegion == POLAR_REGION_OUT) {

    // did we just cross into mid from in?  i.e. end of run
    if (_region == POLAR_REGION_IN) {
      Serial.println("[PM.lA] end of run");
      /*
      calculate effective heading and if close enough to target heading, then record the end location and time, compute the average speed and add to polar info.
      */

      // calc how far we travelled
      float distTravelled = calculateDistanceBetweenCoordinates(
        _startPos[0],
        _startPos[1],
        _subs[POLAR_SUB_LOCATION_E].param.data.f[0],
        _subs[POLAR_SUB_LOCATION_E].param.data.f[1]
      );

      // in how long
      float timeTaken = (millis() - _startTime) / 1000.0;

      // and thus average speed
      float avgSpeed = distTravelled / timeTaken;

      Serial.print("[PM.lA] avgSpeed ");
      Serial.println(avgSpeed);

      // and on what effective heading
      float effectiveHeading = calculateInitialBearingBetweenCoordinates(
        _startPos[0],
        _startPos[1],
        _subs[POLAR_SUB_LOCATION_E].param.data.f[0],
        _subs[POLAR_SUB_LOCATION_E].param.data.f[1]
      );

      Serial.print("[PM.lA] effectiveHeading ");
      Serial.println(effectiveHeading);

      // calc leeway
      float leeway = fabs(shortestSignedDistanceBetweenCircularValues(effectiveHeading, _params[POLAR_PARAM_ADJ_HEADING_E].data.f[0]));

      Serial.print("[PM.lA] leeway ");
      Serial.println(leeway);

      // if leeway less than threshold
      // and we've travelled at least the target radius in distance
      if (leeway < _params[POLAR_PARAM_THRESHOLD_E].data.f[1] &&
         distTravelled > _params[POLAR_PARAM_RADIUS_E].data.f[0]) {

        // scale speed relative to wind speed
        float speedRatio = avgSpeed / _subs[POLAR_SUB_WIND_SPEED_E].param.data.f[0];

        // calc polar bin relative to wind
        float relHeading = shortestSignedDistanceBetweenCircularValues(_params[POLAR_PARAM_ADJ_HEADING_E].data.f[0],
        _subs[POLAR_SUB_WIND_E].param.data.f[0]);
        relHeading = fmod(relHeading,360);
        uint8_t bin = polarIndexForAngle(relHeading);

        // add run to polar
        _polarVals[bin] += speedRatio;
        _params[POLAR_PARAM_SAMPLES_E].data.uint8[bin] += 1;
        publishParamEntry(&_params[POLAR_PARAM_SAMPLES_E]);

        // update polar
        updatePolar();

        // TODO - store leeway

      } else {
        // invalid run

      }
    }

    /*
    turn onto a heading that will orbit the target in a clockwise direction (configurable?) at distance of mid threshold, once speed over ground is over threshold, then select a new heading that goes from current location through the center of the target.
    */
    if (_subs[POLAR_SUB_SOG_E].param.data.f[0] < _params[POLAR_PARAM_THRESHOLD_E].data.f[0]) {
      // SOG too slow... so try to keep on orbiting clockwise along mid-radius
      Serial.println("[PM.lA] SOG too slow, orbit ");

      // chase a point 10 degrees clockwise of where we are now
      GeographicPoint target;
      target.lon = _params[POLAR_PARAM_TARGET_E].data.f[0];
      target.lat = _params[POLAR_PARAM_TARGET_E].data.f[1];
      float bearing = calculateInitialBearingBetweenCoordinates(
        _params[POLAR_PARAM_TARGET_E].data.f[0],
        _params[POLAR_PARAM_TARGET_E].data.f[1],
        _subs[POLAR_SUB_LOCATION_E].param.data.f[0],
        _subs[POLAR_SUB_LOCATION_E].param.data.f[1]
      );
      bearing += 10;

      Serial.print("[PM.lA] bearing ");
      Serial.println(bearing);

      GeographicPoint targetPoint = calculateDestinationFromDistanceAndBearing(target, _params[POLAR_PARAM_RADIUS_E].data.f[1], bearing);

      Serial.print("[PM.lA] destination ");
      Serial.print(targetPoint.lon);
      Serial.print(" ");
      Serial.println(targetPoint.lat);

      // now calculate heading from current location to targetPoint
      float heading = calculateInitialBearingBetweenCoordinates(
        _subs[POLAR_SUB_LOCATION_E].param.data.f[0],
        _subs[POLAR_SUB_LOCATION_E].param.data.f[1],
        targetPoint.lon,
        targetPoint.lat
      );

      Serial.print("[PM.lA] heading ");
      Serial.println(heading);

      // and publish the new heading
      updateAndPublishParam(&_params[POLAR_PARAM_ADJ_HEADING_E], (uint8_t*)&heading, sizeof(heading));



    } else {
      // SOG good... turn onto new heading through target centre
      //Serial.println("[PM.lA] SOG Good ");

      float newHeading = calculateInitialBearingBetweenCoordinates(
        _subs[POLAR_SUB_LOCATION_E].param.data.f[0],
        _subs[POLAR_SUB_LOCATION_E].param.data.f[1],
        _params[POLAR_PARAM_TARGET_E].data.f[0],
        _params[POLAR_PARAM_TARGET_E].data.f[1]
      );

      // add a few degrees to the heading so the boat travels in a pointed star pattern
      //newHeading += 15;
      //newHeading = fmod(newHeading,360);

      // search the polar bins around the tentative new heading and see if there's an empty bin close by
      float relHeading = newHeading - _subs[POLAR_SUB_WIND_E].param.data.f[0];
      relHeading = fmod(relHeading,360);
      if (relHeading < 0) relHeading += 360;
      
      uint8_t pi = polarIndexForAngle(relHeading);
      int i1 = pi-2;
      if (i1 < 3) i1=3;
      int i2 = pi+2;
      if (i2 > 15) i2 = 15;

      for (uint8_t i=i1; i<i2+1; i++) {
        if (_params[POLAR_PARAM_SAMPLES_E].data.uint8[i] < _params[POLAR_PARAM_SAMPLES_E].data.uint8[pi]) {
          // update pi
          pi = i;
        }
      }

      // calculate newHeading based on pi
      relHeading = (relHeading > 180) ? 360 - pi * 11.25 : pi * 11.25;
      newHeading = relHeading + _subs[POLAR_SUB_WIND_E].param.data.f[0];
      newHeading = fmod(newHeading,360);
      if (newHeading < 0) newHeading += 360;


      //Serial.print("[PM.lA] newHeading ");
      //Serial.println(newHeading);

      updateAndPublishParam(&_params[POLAR_PARAM_ADJ_HEADING_E], (uint8_t*)&newHeading, sizeof(newHeading));
    }
  }


  // if in the inner region...
  if (newRegion == POLAR_REGION_IN) {
    // did we just cross into in from mid?  i.e. start of run
    if (_region == POLAR_REGION_MID) {
      /*
      record the start location and time.
      */
      _startTime = millis();
      _startPos[0] = _subs[POLAR_SUB_LOCATION_E].param.data.f[0];
      _startPos[1] = _subs[POLAR_SUB_LOCATION_E].param.data.f[1];

      Serial.print("[PM.lA] start of run ");
      Serial.println(_startTime);
    }
  }



  _region = newRegion;
}


void PolarModule::loopReset() {
  // clear aggregated values and reset counters
  for (uint8_t i=0; i<16; i++) {
    _polarVals[i] = 0;

    _params[POLAR_PARAM_SAMPLES_E].data.uint8[0] = 0;
    publishParamEntry(&_params[POLAR_PARAM_SAMPLES_E]);

    _params[POLAR_PARAM_POLAR_E].data.uint8[0] = 0;
    publishParamEntry(&_params[POLAR_PARAM_POLAR_E]);
  }

  // switch mode to active
  uint8_t newMode = POLAR_MODE_ACTIVE;
  updateAndPublishParam(&_params[POLAR_PARAM_MODE_E], (uint8_t*)&newMode, sizeof(newMode));
}
