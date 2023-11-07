#include "SailorModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../strings.h"
#include "../navMath.h"

// @type Sailor

// return true if a is between b and c
// used to evaluate if changing course from b to c will cross the wind a
boolean isAngleBetweenAngles(float a, float b, float c) {
  float bToC = shortestSignedDistanceBetweenCircularValues(b, c);
  float bToA = shortestSignedDistanceBetweenCircularValues(b, a);

  // will cross wind if sign of bToA matches bToC AND magnitude of bToA < bToC
  boolean sameSign = (bToC * bToA) > 0;
  return (sameSign && (fabs(bToA) < fabs(bToC)));
}

SailorModule::SailorModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   // set type
   setTypeName(FPSTR(SAILOR_STR_SAILOR));

   _courseWind = 0;
   _courseTarget = 0;

   _lastUpdate = 0;
   _iError = 0;
   _dError = 0;
   _lastError = 0;
   _lastHeading = 0;

   _gybeTimerStart = 0;
   _positiveError = false;

   _potentialStall = false;
   _stallTimerStart = 0;

   // subs
   initSubs(SAILOR_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[SAILOR_SUB_TARGET_E];
   sub->addrParam = SAILOR_SUB_TARGET_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SAILOR_SUB_TARGET);
   setParamName(FPSTR(STRING_TARGET), &sub->param);

   sub = &_subs[SAILOR_SUB_HEADING_E];
   sub->addrParam = SAILOR_SUB_HEADING_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SAILOR_SUB_HEADING);
   setParamName(FPSTR(STRING_HEADING), &sub->param);

   sub = &_subs[SAILOR_SUB_WIND_E];
   sub->addrParam = SAILOR_SUB_WIND_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SAILOR_SUB_WIND);
   setParamName(FPSTR(STRING_WIND), &sub->param);

   sub = &_subs[SAILOR_SUB_CROSSTRACK_E];
   sub->addrParam = SAILOR_SUB_CROSSTRACK_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, SAILOR_SUB_CROSSTRACK);
   setParamName(FPSTR(STRING_CROSSTRACK), &sub->param);

   // pubs
   initParams(SAILOR_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[SAILOR_PARAM_COURSE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, SAILOR_PARAM_COURSE);
   setParamName(FPSTR(STRING_COURSE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[SAILOR_PARAM_SHEET_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, SAILOR_PARAM_SHEET);
   setParamName(FPSTR(STRING_SHEET), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[SAILOR_PARAM_POLAR_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SAILOR_PARAM_POLAR);
   setParamName(FPSTR(STRING_POLAR), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 16);

   param = &_params[SAILOR_PARAM_SPEED_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SAILOR_PARAM_SPEED);
   //param->publish = true;
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);

   param = &_params[SAILOR_PARAM_SPEED2_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SAILOR_PARAM_SPEED2);
   //param->publish = true;
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);

   param = &_params[SAILOR_PARAM_FLAGS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, SAILOR_PARAM_FLAGS);
   //param->publish = true;
   setParamName(FPSTR(STRING_FLAGS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 5);
   param->data.uint8[SAILOR_FLAG_STATE] = SAILOR_STATE_PLANNING;
   param->data.uint8[SAILOR_FLAG_TACK] = SAILOR_TACK_UNDEFINED;
   param->data.uint8[SAILOR_FLAG_GYBE] = SAILOR_GYBE_NORMAL;
   param->data.uint8[SAILOR_FLAG_COURSE_WIND] = 0;
   param->data.uint8[SAILOR_FLAG_CROSS_THE_WIND] = 0;

   param = &_params[SAILOR_PARAM_WING_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, SAILOR_PARAM_WING);
   setParamName(FPSTR(STRING_WING), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);


   // rudder control
   param = &_params[SAILOR_PARAM_PID_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SAILOR_PARAM_PID);
   setParamName(FPSTR(STRING_PID), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   // @default PID=0.05,0,0
   param->data.f[0] = 0.05;
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[SAILOR_PARAM_RUDDER_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, SAILOR_PARAM_RUDDER);
   setParamName(FPSTR(STRING_RUDDER), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[SAILOR_PARAM_THRESHOLD_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SAILOR_PARAM_THRESHOLD);
   setParamName(FPSTR(STRING_THRESHOLD), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   // @default threshold=20
   param->data.f[0] = 20;

   param = &_params[SAILOR_PARAM_TIMEOUT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SAILOR_PARAM_TIMEOUT);
   setParamName(FPSTR(STRING_TIMEOUT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   // @default timeout=10
   param->data.f[0] = 10;

   param = &_params[SAILOR_PARAM_MODE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, SAILOR_PARAM_MODE);
   setParamName(FPSTR(STRING_MODE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   param->data.uint8[0] = SAILOR_MODE_NORMAL;

   update();  // set defaults
}


uint8_t SailorModule::polarIndexForAngle(float ang) {
  //float w = _subs[SAILOR_SUB_WIND_E].param.data.f[0];

  //float polarAng = fmod(ang - w, 360);
  float polarAng = fmod(ang, 360);
  if (polarAng < 0) polarAng += 360;
  uint8_t polarIndex = polarAng / 11.25;
  if (polarIndex > 31) polarIndex -= 32;

  return polarIndex;
}


uint8_t SailorModule::polarForAngle(float ang) {
  uint8_t polarIndex = polarIndexForAngle(ang);
  if (polarIndex > 15) polarIndex = 31 - polarIndex;
  return _params[SAILOR_PARAM_POLAR_E].data.uint8[polarIndex];
}


void SailorModule::update() {
  DroneModule::update();
  if (!_setupDone) return;

}


void SailorModule::loop() {
  DroneModule::loop();

  /*
  Notes
   * negative cross-track means the target is to the left, i.e. we are to the right of the corridor
   * positive crossdtrack means targe to the right, we are to the left of the corridor

  */

  // local shortcuts
  float h = _subs[SAILOR_SUB_HEADING_E].param.data.f[0];
  float w = _subs[SAILOR_SUB_WIND_E].param.data.f[0];
  float t = _subs[SAILOR_SUB_TARGET_E].param.data.f[0];
  float ct = _subs[SAILOR_SUB_CROSSTRACK_E].param.data.f[0];
  float c = _params[SAILOR_PARAM_COURSE_E].data.f[0];

  uint8_t newState = _params[SAILOR_PARAM_FLAGS_E].data.uint8[SAILOR_FLAG_STATE];
  uint8_t newTack = _params[SAILOR_PARAM_FLAGS_E].data.uint8[SAILOR_FLAG_TACK];
  boolean willCrossWind = _params[SAILOR_PARAM_FLAGS_E].data.uint8[SAILOR_FLAG_CROSS_THE_WIND] == 1;

  uint8_t newGybe = _params[SAILOR_PARAM_FLAGS_E].data.uint8[SAILOR_FLAG_GYBE];

  // calc sheet based on delta between heading and wind
  float wing = shortestSignedDistanceBetweenCircularValues(h, w);
  float sheet = fabs(wing) / 180;
  if (sheet > 1) sheet = 1;


  if (newState == SAILOR_STATE_PLANNING) {
    /*
      we need to select a new course
    */ 
   _courseTarget = t;
   _courseWind = w;

    float maxPV = -10;
    float newC = c;

    // sweep around potential headings relative to wind (i.e. polar coord frame)
    for (uint8_t i=0; i<32; i++) {
      float ang = (i * 360.0/32.0) + 360.0/64.0;
      float worldAng = ang + w;
      float deltaToTarget = fabs((worldAng) - t);
      // dot the polar performance into the target vector, may include negative values
      float pv = cos(degreesToRadians(deltaToTarget)) * polarForAngle(ang);
      boolean penalise = false;

      // if we're at the edge of the tacking corridor
      if (fabs(ct) > 0.8) {
        // penalise courses on wrong side of tacking corridor
        // delta from 
        float pToT = shortestSignedDistanceBetweenCircularValues(t, worldAng);
        if (ct > 0) {
          // on left side of corridor
          //penalise = (pToT < 0 || pToT > 90);
          penalise = (pToT < 0 || pToT > 180);
        } else {
          // on right side of corridor
          //penalise = (pToT > 0 || pToT < -90);
          penalise = (pToT > 0 || pToT < -180);
        }
      }
      
      if (penalise) pv = -10;   // pv / 50

      if (i < 16) {
        _params[SAILOR_PARAM_SPEED_E].data.uint8[i] = max(pv, 0.0f);
      } else {
        _params[SAILOR_PARAM_SPEED2_E].data.uint8[i-16] = max(pv, 0.0f);
      }
      if (pv > maxPV) {
        maxPV = pv;
        newC = ang + w; // put back into world frame
        if (ct > 0) {
          // currently on port side of corridor, so we should have selected a port tack
          newTack = SAILOR_TACK_PORT;
        } else {
          newTack = SAILOR_TACK_STARBOARD;
        }
      }
    }
    
    // ensure course is in range 0.. 360
    newC = fmod(newC, 360);
    if (newC < 0) newC += 360;

    // if course is close to target, then just head to target
    if (fabs(newC-t) < 11) {
      newC = t;
    }

    // determine if new course will require crossing the wind
    willCrossWind = isAngleBetweenAngles(w, c, newC);

    // update c value for use in rudder control
    c = newC;

    updateAndPublishParam(&_params[SAILOR_PARAM_COURSE_E], (uint8_t*)&c, sizeof(c));

    publishParamEntry(&_params[SAILOR_PARAM_SPEED_E]);
    publishParamEntry(&_params[SAILOR_PARAM_SPEED2_E]);

    newState = SAILOR_STATE_COURSE_SET;

    // reset any gybe in progress
    newGybe = SAILOR_GYBE_NORMAL;

    // reset stall timer
    _potentialStall = false;

  } else {
    /* 
       a course is set... not much todo except check if we need to set a new course
    */
    boolean replan = false;

    if (newState == SAILOR_STATE_COURSE_UNDERWAY) {
      // have we reached the edge of the tacking corridor
      if (fabs(ct) >= 1) {
        replan = true;
      }

      // is the wind now blowing in a direction where we are likely to stall?
      // get current expected polar performance
      float cpv = polarForAngle(h - w);
      if (cpv == 0) {
        // start stall timer
        if (_potentialStall) {
          if (millis() > _stallTimerStart + 5000) {
            replan = true;
            _potentialStall = false;
          }

        } else {
          _potentialStall = true;
          _stallTimerStart = millis();
        }
      } else {
        // cancel potential stall
        _potentialStall = false;
      }
    } else {
      // have we progressed far on this new course to consider ourselves underway?
      // and have we managed to get on the intended course
      float headingError = shortestSignedDistanceBetweenCircularValues( h, c );
      if ((fabs(ct) < 0.8) && (fabs(headingError) < _params[SAILOR_PARAM_THRESHOLD_E].data.f[0])) {
        newState = SAILOR_STATE_COURSE_UNDERWAY;
      }
    }
    
    // has wind direction changed dramatically since we selected a course
    /*
    float windDelta = shortestSignedDistanceBetweenCircularValues(w, _courseWind);
    if (fabs(windDelta) > 40) { 
      replan = true;
    }
    */


    // are we well beyond the edge of the tacking corridor
    if (fabs(ct) >= 1.5) {
      replan = true;
    }

  
    // has the target heading changed dramatically since we selected a course
    float targetDelta = shortestSignedDistanceBetweenCircularValues(t, _courseTarget);
    if (fabs(targetDelta) > 80) { 
      replan = true;
    }
   
    if (replan) {
      newState = SAILOR_STATE_PLANNING;
    }
  }


  // remap sheet in range -1 to 1
  sheet = (sheet * 2) - 1;
  updateAndPublishParam(&_params[SAILOR_PARAM_SHEET_E], (uint8_t*)&sheet, sizeof(sheet));

  // rescale
  wing = wing > 0 ? 1 : -1;
  updateAndPublishParam(&_params[SAILOR_PARAM_WING_E], (uint8_t*)&wing, sizeof(wing));

  //publishParamEntry(&_params[SAILOR_PARAM_WING_E]);

  /*
    Rudder Control
  */

 // target should be the planned course
 t = c;

  unsigned long updateTime = millis();
  float dt = (updateTime - _lastUpdate) / 1000.0;

  // don't bother updating if dt too small
  if (dt < 0.05) return;

  _lastUpdate = updateTime;

  // check to see if heading has dramatically changed
  if (fabs(shortestSignedDistanceBetweenCircularValues(_lastHeading, t)) > 45) {
    // reset d and i errors
    _iError = 0;
    _dError = 0;
  }

  // calc shortest signed distance
  // positive values indicate a clockwise turn
  float err = shortestSignedDistanceBetweenCircularValues( h, t );
  boolean positiveError = err > 0;

  // check for potential gybe condition
  if (fabs(err) > _params[SAILOR_PARAM_THRESHOLD_E].data.f[0]) {
    if (newGybe == SAILOR_GYBE_NORMAL) {
      // if in normal mode, then change to potential gybe and start the timer
      newGybe = SAILOR_GYBE_POTENTIAL;
      _gybeTimerStart = updateTime;
      _positiveError = err > 0;
    }

    if (positiveError != _positiveError) {
      // reset to normal mode
      //newGybe = SAILOR_MODE_NORMAL;
      // dont reset to normal until error falls below threshold... i.e. gybe complete
    //} else if (cpv < 50) { 
      // we appear to be in a very low performance region of the polar
      // and may be stuck here


    } else {
      if ( (updateTime - _gybeTimerStart)/1000.0 > _params[SAILOR_PARAM_TIMEOUT_E].data.f[0]) {
        // if in potential gybe and timer has exceed the timeout... switch to gybe mode
        newGybe = SAILOR_GYBE_GYBING;
      }
    } 
  } else {
    // reset to normal mode
    newGybe = SAILOR_GYBE_NORMAL;
  }

  // if gybe mode then invert err
  if (newGybe == SAILOR_GYBE_GYBING) {
    /*
    // if _positiveError then when we first entered gybe we needed to turn clockwise to reach the targetHeading
    if ((err > 0 && _positiveError) || (err < 0 && !_positiveError)) {
      //err = -err;
      // make sure err is large for maximum rudder authority
      err = err > 0 ? -90 : 90;
    }
    */

    // if we are gybing, we need to choose a turn that gets us to the target without crossing the wind
    // first lets check to see if the natural direction would cross the wind
    if (isAngleBetweenAngles(w, h, t)) {
      // we need to take the opposite way round to reach the target, so invert and enlarge the err value
      if (err > 0) {
        err = -90;
      } else {
        err = 90;
      }
    }
  }

  // limit to 90 for ease
  if (err > 90) err = 90;
  if (err < -90) err = -90;

  // update I and D terms
  _iError += err * dt;
  _dError = (err - _lastError) / dt;

  // clamp i error
  if (_params[SAILOR_PARAM_PID_E].data.f[1] > 0) {
    if (fabs(_iError) > 100 / _params[SAILOR_PARAM_PID_E].data.f[1]) {
      _iError = (_iError > 0 ? 100 : -100) / _params[SAILOR_PARAM_PID_E].data.f[1];
    }
  }

  // apply PID cooefficients
  float tr =
    err * _params[SAILOR_PARAM_PID_E].data.f[0] +
    _iError * _params[SAILOR_PARAM_PID_E].data.f[1] +
    _dError * _params[SAILOR_PARAM_PID_E].data.f[2];

  _lastError = err;

  // apply limits
  if (tr > 1) tr = 1;
  if (tr < -1) tr = -1;

  updateAndPublishParam(&_params[SAILOR_PARAM_RUDDER_E], (uint8_t*)&tr, sizeof(tr));


  uint8_t flags[4];
  flags[SAILOR_FLAG_STATE] = newState;
  flags[SAILOR_FLAG_TACK] = newTack;
  flags[SAILOR_FLAG_GYBE] = newGybe;
  flags[SAILOR_FLAG_COURSE_WIND] = round(255 * _courseWind / 360); 
  flags[SAILOR_FLAG_CROSS_THE_WIND] = willCrossWind ? 1 : 0;

  updateAndPublishParam(&_params[SAILOR_PARAM_FLAGS_E], (uint8_t*)&flags, sizeof(flags));

  _lastHeading = h;

}
