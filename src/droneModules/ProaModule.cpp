#include "ProaModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../strings.h"
#include "../navMath.h"

ProaModule::ProaModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm , dem, fs)
 {
   // set type
   setTypeName(FPSTR(PROA_STR_PROA));

   _starboardTack = true;
   _tackLocked = false;
   _lastCrossTrackPositive = false;
   //_wingAOA = 8; // optimal angle of attack for Eppler e169 profile
   // http://airfoiltools.com/airfoil/details?airfoil=e169-il

   // subs
   initSubs(PROA_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[PROA_SUB_TARGET_E];
   sub->addrParam = PROA_SUB_TARGET_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_SUB_TARGET);
   setParamName(FPSTR(STRING_TARGET), &sub->param);

   sub = &_subs[PROA_SUB_HEADING_E];
   sub->addrParam = PROA_SUB_HEADING_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_SUB_HEADING);
   setParamName(FPSTR(STRING_HEADING), &sub->param);

   sub = &_subs[PROA_SUB_WIND_E];
   sub->addrParam = PROA_SUB_WIND_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_SUB_WIND);
   setParamName(FPSTR(STRING_WIND), &sub->param);

   sub = &_subs[PROA_SUB_CROSSTRACK_E];
   sub->addrParam = PROA_SUB_CROSSTRACK_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_SUB_CROSSTRACK);
   setParamName(FPSTR(STRING_CROSSTRACK), &sub->param);

   sub = &_subs[PROA_SUB_COW_E];
   sub->addrParam = PROA_SUB_COW_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_SUB_COW);
   setParamName(FPSTR(STRING_COW), &sub->param);

   // pubs
   initParams(PROA_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[PROA_PARAM_COURSE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_PARAM_COURSE);
   setParamName(FPSTR(STRING_COURSE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[PROA_PARAM_WING_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_PARAM_WING);
   setParamName(FPSTR(STRING_WING), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[PROA_PARAM_POLAR_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_PARAM_POLAR);
   setParamName(FPSTR(STRING_POLAR), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 16);

   param = &_params[PROA_PARAM_SPEED_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_PARAM_SPEED);
   param->publish = true;
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);

   param = &_params[PROA_PARAM_SPEED2_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_PARAM_SPEED2);
   param->publish = true;
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);
   for (uint8_t i=0; i<16; i++) {
     _params[PROA_PARAM_SPEED_E].data.uint8[i] = 0;
     _params[PROA_PARAM_SPEED2_E].data.uint8[i] = 0;
   }

   param = &_params[PROA_PARAM_FLAGS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, PROA_PARAM_FLAGS);
   param->publish = true;
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 3);

   param = &_params[PROA_PARAM_LEFT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, PROA_PARAM_LEFT);
   setParamName(FPSTR(STRING_LEFT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[PROA_PARAM_RIGHT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, PROA_PARAM_RIGHT);
   setParamName(FPSTR(STRING_RIGHT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[PROA_PARAM_PID_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_PARAM_PID);
   setParamName(FPSTR(STRING_PID), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[PROA_PARAM_PID_E].data.f[0] = 0.5;
   _params[PROA_PARAM_PID_E].data.f[1] = 0;
   _params[PROA_PARAM_PID_E].data.f[2] = 0;

   param = &_params[PROA_PARAM_AOA_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_PARAM_AOA);
   setParamName(FPSTR(STRING_AOA), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[PROA_PARAM_AOA_E].data.f[0] = 8;

   param = &_params[PROA_PARAM_OFFSET_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_PARAM_OFFSET);
   setParamName(FPSTR(STRING_OFFSET), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[PROA_PARAM_MODE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PROA_PARAM_MODE);
   setParamName(FPSTR(STRING_MODE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[PROA_PARAM_MODE_E].data.f[0] = 0;

   update();  // set defaults
}


DEM_NAMESPACE* ProaModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(PROA_STR_PROA,0,true);
}

void ProaModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_TARGET, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$target"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_HEADING, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$heading"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_WIND, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$wind"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_CROSSTRACK, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$crosstrack"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_COW, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$COW"), DRONE_LINK_MSG_TYPE_ADDR, pha);

  dem->registerCommand(ns, STRING_POLAR, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_PID, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_AOA, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_MODE, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
}


uint8_t ProaModule::polarIndexForAngle(float ang) {
  //float w = _subs[PROA_SUB_WIND_E].param.data.f[0];

  //float polarAng = fmod(ang - w, 360);
  float polarAng = fmod(ang, 360);
  if (polarAng < 0) polarAng += 360;
  uint8_t polarIndex = polarAng / 11.25;
  if (polarIndex > 31) polarIndex -= 32;

  return polarIndex;
}


uint8_t ProaModule::polarForAngle(float ang) {
  uint8_t polarIndex = polarIndexForAngle(ang);
  if (polarIndex > 15) polarIndex = 31 - polarIndex;
  return _params[PROA_PARAM_POLAR_E].data.uint8[polarIndex];
}


void ProaModule::update() {
  DroneModule::update();
  if (!_setupDone) return;

  /*
  Mode check
  */

  if (_params[PROA_PARAM_MODE_E].data.uint8[0] == 0) {
    // in setup/idle mode

    // set all servos to centre positions
    float tempF = 0;
    updateAndPublishParam(&_params[PROA_PARAM_WING_E], (uint8_t*)&tempF, sizeof(tempF));
    updateAndPublishParam(&_params[PROA_PARAM_LEFT_E], (uint8_t*)&tempF, sizeof(tempF));
    updateAndPublishParam(&_params[PROA_PARAM_RIGHT_E], (uint8_t*)&tempF, sizeof(tempF));

    return;
  }

  /*
   * negative cross-track means the target is to the left, positive to the right
  */

  // local shortcuts
  float h = _subs[PROA_SUB_HEADING_E].param.data.f[0];
  float w = _subs[PROA_SUB_WIND_E].param.data.f[0];
  float t = _subs[PROA_SUB_TARGET_E].param.data.f[0];
  float ct = _subs[PROA_SUB_CROSSTRACK_E].param.data.f[0];
  float c = _params[PROA_PARAM_COURSE_E].data.f[0];
  float cow = _subs[PROA_SUB_COW_E].param.data.f[0];
  float aoa = _params[PROA_PARAM_AOA_E].data.f[0];

  // -- algo 1 --
  // evaluate all potential course optionsl, using the polar for each
  // dot the polar value into the target vector
  // pick the one with the largest vector magnitude

  /*
  if abs(crosstrack > 1) then its time to tack
     when evaluating potential headings, need to rule out headings that would increase cross-track
     i.e. zero the pv values
     if crosstrack positive, then relative headings need to be between 0 and 90 degrees
     if crosstrack negative, then relative headings need to be between 0 and -90 degrees
     where relative heading is shortestSignedDistanceBetweenCircularValues(target, potential heading)
  */

  if (_tackLocked && (_lastCrossTrackPositive ? ct < -1 : ct > 1)) _tackLocked = false;
  if (fabs(ct) > 1.2) {
    _tackLocked = false;
  }


  float maxPV = 0;
  float newTackIsStarboard = true;
  // sweep around potential headings relative to wind (i.e. polar coord frame)
  for (uint8_t i=0; i<32; i++) {
    float ang = (i * 360.0/32.0) + 360.0/64.0;
    float deltaToTarget = fabs((ang + w) - t);
    // dot the polar performance into the target vector
    float pv = cos(degreesToRadians(deltaToTarget)) * polarForAngle(ang);
    if (pv < 0) pv = 0;
    boolean penalise = false;

    if (_tackLocked) {
      // penalise tacking before its needed
      if (i < 16) {
        // port wind
        penalise = _starboardTack;
      } else {
        // starboard wind
        penalise = !_starboardTack;
      }
    }

    if (fabs(ct) > 1) {
      float pToT = shortestSignedDistanceBetweenCircularValues(t, ang + w);
      if (ct > 0) {
        penalise = (pToT < 0 || pToT > 90);
      } else {
        penalise = (pToT > 0 || pToT < -90);
      }
    }

    if (penalise) pv = pv / 50;

    if (i < 16) {
      _params[PROA_PARAM_SPEED_E].data.uint8[i] = pv;
    } else {
      _params[PROA_PARAM_SPEED2_E].data.uint8[i-16] = pv;
    }
    if (pv > maxPV) {
      maxPV = pv;
      c = ang + w; // put back into world frame
      newTackIsStarboard = i > 15;
    }
  }

  // if course is close to target, then just head to target
  if (fabs(c-t) < 11) {
    c = t;
  }

  // see if we're on a rubbish tack
  if (maxPV < 25) {
    _tackLocked = false;
  }

  boolean tackChanged = false;

  // see if we need to force a tack
  // should never happen
  if (fabs(ct) > 1.2) {
    _starboardTack = ct < 0;
    _tackLocked = true;
    _lastCrossTrackPositive = ct > 0;
    tackChanged = true;
  }

  // see if we're changing tack
  //if (!_tackLocked && newTackIsStarboard != _starboardTack && fabs(ct)<1.2) {
  if (newTackIsStarboard != _starboardTack && fabs(ct)<1.2) {
    _starboardTack = newTackIsStarboard;
    _tackLocked = true;
    _lastCrossTrackPositive = ct > 0;
    tackChanged = true;
  }


  uint8_t flags[3];
  flags[0] = _starboardTack ? 1 : 0;
  flags[1] = _tackLocked ? 1 : 0;
  flags[2] = _lastCrossTrackPositive ? 1 : 0;

  updateAndPublishParam(&_params[PROA_PARAM_FLAGS_E], (uint8_t*)&flags, sizeof(flags));

  updateAndPublishParam(&_params[PROA_PARAM_COURSE_E], (uint8_t*)&c, sizeof(c));

  // ---------------------------------------------------------------------------
  // update the wing position based on where the wind is coming from
  // i.e. to allow for us being in the wrong orientation, or mid tack
  float localWind = w - h;
  float wingAng = 0;
  if (localWind < 0) localWind += 360;
  if (localWind < 90 || localWind > 270) {
    // wind over bow
    // set wing for maximum drag (orthogonal to localWind)
    if (localWind < 90) {
      // starboard wind, wing should be to port (servo -1 .. 0)
      wingAng = localWind - 90;
    } else {
      // port wind, wing should be to starboard (servo 0 .. 1)
      wingAng = localWind + 90;
    }
  } else {
    // wind over stern
    if (localWind < 180) {
      // starboard wind
      wingAng = (localWind - 180) - aoa;
    } else {
      // port wind
      wingAng = (localWind - 180) + aoa;
    }
  }

  // map wingAng to -90 to 90
  if (wingAng > 180) { wingAng -= 360; }
  // clamp
  if (wingAng > 90) wingAng = 90;
  if (wingAng < -90) wingAng = -90;

  // remap wingAng to -1 to 1
  wingAng = wingAng / 90;
  updateAndPublishParam(&_params[PROA_PARAM_WING_E], (uint8_t*)&wingAng, sizeof(wingAng));

  // ---------------------------------------------------------------------------
  // if we've changed tack, need to recalculate the target frame orientation
  // target frame orientation is an OFFSET relative to the target heading

  // OR course has changed... so just recalc every time
  if (tackChanged || true) {
    float offset = 0;

    if (_starboardTack) {
      // frame orientation should be
      float frameOrientation = w - 90 - aoa;
      offset = frameOrientation - c;
    } else {
      // frame orientation should be
      float frameOrientation = w + 90 + aoa;
      offset = frameOrientation - c;
    }

    updateAndPublishParam(&_params[PROA_PARAM_OFFSET_E], (uint8_t*)&offset, sizeof(offset));
  }

  // ---------------------------------------------------------------------------
  // calculate the error in orientation between current heading (h), adjusted for frame offset, and selected course (c)
  float err = shortestSignedDistanceBetweenCircularValues(h - _params[PROA_PARAM_OFFSET_E].data.f[0], c);

  // calculate the error in COW vs intended course
  float cowErr = shortestSignedDistanceBetweenCircularValues(cow, c);

  // use the orientation error and COW to set the pontoon positions

  // -- left -----------------------
  float la = cow;

  // left pontoon only steers for CCW turns (i.e. err < 0)
  if (err < 0) {
    // Left pontoon toe in if lagging, toe out if leading.
    // lagging is when COW is 0 - 180
    // leading is when COW is 180 - 360
    // toe in is a positive err
    // toe out is a negative err
    boolean toeIn = false;
    if (cow < 180) toeIn = true;

    // PID to control angle
    la += (toeIn ? 1 : -1) * _params[PROA_PARAM_PID_E].data.f[0] * err;
  } else {
    // this is the leading pontoon, so adjust its position to align COW with c
    // negative coeErr = toe Out
    // positive coeErr = toe In
    // use half the "power" vs the trailing pontoon
    la += _params[PROA_PARAM_PID_E].data.f[0] * 0.5 * cowErr;
  }

  // ensure in range 0..360
  la = fmod(la, 360);
  // remap to +-180
  if (la > 180) la -= 360;
  // map -90 to 90
  if (la > 90) la -= 180;
  if (la < -90) la += 180;
  // remap -1 to 1
  la = la / 90;

  updateAndPublishParam(&_params[PROA_PARAM_LEFT_E], (uint8_t*)&la, sizeof(la));

  // -- right -----------------------
  float ra = cow;

  // right pontoon only steers for CW turns (i.e. err > 0)
  if (err > 0) {
    // Right pontoon toe in if lagging, toe out if leading.
    // lagging is when COW is 180 - 360
    // leading is when COW is 0 - 180
    // toe in is a negative err
    // toe out is a positive err
    boolean toeIn = false;
    if (cow > 180) toeIn = true;

    // PID to control angle
    ra += (toeIn ? -1 : 1) * _params[PROA_PARAM_PID_E].data.f[0] * err;
  } else {
    // this is the leading pontoon, so adjust its position to align COW with c
    // negative coeErr = toe Out
    // positive coeErr = toe In
    // use half the "power" vs the trailing pontoon
    ra += _params[PROA_PARAM_PID_E].data.f[0] * 0.5 * cowErr;
  }

  // ensure in range 0..360
  ra = fmod(ra, 360);
  // remap to +-180
  if (ra > 180) ra -= 360;
  // map -90 to 90
  if (ra > 90) ra -= 180;
  if (ra < -90) ra += 180;
  // remap -1 to 1
  ra = ra / 90;

  updateAndPublishParam(&_params[PROA_PARAM_RIGHT_E], (uint8_t*)&ra, sizeof(ra));
}


void ProaModule::loop() {
  DroneModule::loop();
  publishParamEntry(&_params[PROA_PARAM_SPEED_E]);
  publishParamEntry(&_params[PROA_PARAM_SPEED2_E]);
}
