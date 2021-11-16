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
   _wingAOA = 10; // angle of attack

   // subs
   initSubs(PROA_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[PROA_SUB_TARGET_E];
   sub->addrParam = PROA_SUB_TARGET_ADDR;
   sub->param.param = PROA_SUB_TARGET;
   setParamName(FPSTR(STRING_TARGET), &sub->param);

   sub = &_subs[PROA_SUB_HEADING_E];
   sub->addrParam = PROA_SUB_HEADING_ADDR;
   sub->param.param = PROA_SUB_HEADING;
   setParamName(FPSTR(STRING_HEADING), &sub->param);

   sub = &_subs[PROA_SUB_WIND_E];
   sub->addrParam = PROA_SUB_WIND_ADDR;
   sub->param.param = PROA_SUB_WIND;
   setParamName(FPSTR(STRING_WIND), &sub->param);

   sub = &_subs[PROA_SUB_CROSSTRACK_E];
   sub->addrParam = PROA_SUB_CROSSTRACK_ADDR;
   sub->param.param = PROA_SUB_CROSSTRACK;
   setParamName(FPSTR(STRING_CROSSTRACK), &sub->param);

   // pubs
   initParams(PROA_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[PROA_PARAM_COURSE_E];
   param->param = PROA_PARAM_COURSE;
   setParamName(FPSTR(STRING_COURSE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[PROA_PARAM_WING_E];
   param->param = PROA_PARAM_WING;
   setParamName(FPSTR(STRING_WING), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[PROA_PARAM_POLAR_E];
   param->param = PROA_PARAM_POLAR;
   setParamName(FPSTR(STRING_POLAR), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 16);

   param = &_params[PROA_PARAM_SPEED_E];
   param->param = PROA_PARAM_SPEED;
   param->publish = true;
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);

   param = &_params[PROA_PARAM_SPEED2_E];
   param->param = PROA_PARAM_SPEED2;
   param->publish = true;
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);

   param = &_params[PROA_PARAM_FLAGS_E];
   param->param = PROA_PARAM_FLAGS;
   param->publish = true;
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 3);

   param = &_params[PROA_PARAM_LEFT_E];
   param->param = PROA_PARAM_LEFT;
   setParamName(FPSTR(STRING_LEFT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[PROA_PARAM_RIGHT_E];
   param->param = PROA_PARAM_RIGHT;
   setParamName(FPSTR(STRING_RIGHT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

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

  dem->registerCommand(ns, STRING_POLAR, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
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
   * negative cross-track means the target is to the left, positive to the right
  */

  // local shortcuts
  float h = _subs[PROA_SUB_HEADING_E].param.data.f[0];
  float w = _subs[PROA_SUB_WIND_E].param.data.f[0];
  float t = _subs[PROA_SUB_TARGET_E].param.data.f[0];
  float ct = _subs[PROA_SUB_CROSSTRACK_E].param.data.f[0];
  float c = _params[PROA_PARAM_COURSE_E].data.f[0];

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

  // see if we need to force a tack
  // should never happen
  if (fabs(ct) > 1.2) {
    _starboardTack = ct < 0;
    _tackLocked = true;
    _lastCrossTrackPositive = ct > 0;
  }

  // see if we're changing tack
  //if (!_tackLocked && newTackIsStarboard != _starboardTack && fabs(ct)<1.2) {
  if (newTackIsStarboard != _starboardTack && fabs(ct)<1.2) {
    _starboardTack = newTackIsStarboard;
    _tackLocked = true;
    _lastCrossTrackPositive = ct > 0;
  }


  uint8_t flags[3];
  flags[0] = _starboardTack ? 1 : 0;
  flags[1] = _tackLocked ? 1 : 0;
  flags[2] = _lastCrossTrackPositive ? 1 : 0;

  updateAndPublishParam(&_params[PROA_PARAM_FLAGS_E], (uint8_t*)&flags, sizeof(flags));

  updateAndPublishParam(&_params[PROA_PARAM_COURSE_E], (uint8_t*)&c, sizeof(c));

  // having decided the cource (c), now we need to set the pontoon positions



  // and finally update the wing position based on where the wind is coming from
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
      wingAng = (localWind - 180) - _wingAOA;
    } else {
      // port wind
      wingAng = (localWind - 180) + _wingAOA;
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
}


void ProaModule::loop() {
  DroneModule::loop();
  publishParamEntry(&_params[PROA_PARAM_SPEED_E]);
  publishParamEntry(&_params[PROA_PARAM_SPEED2_E]);
}
