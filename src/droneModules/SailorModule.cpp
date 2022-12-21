#include "SailorModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../strings.h"
#include "../navMath.h"

SailorModule::SailorModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   // set type
   setTypeName(FPSTR(SAILOR_STR_SAILOR));

   _starboardTack = true;
   _tackLocked = false;
   _lastCrossTrackPositive = false;

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
   param->publish = true;
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);

   param = &_params[SAILOR_PARAM_SPEED2_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SAILOR_PARAM_SPEED2);
   param->publish = true;
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 16);

   param = &_params[SAILOR_PARAM_FLAGS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, SAILOR_PARAM_FLAGS);
   param->publish = true;
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 3);

   param = &_params[SAILOR_PARAM_WING_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, SAILOR_PARAM_WING);
   setParamName(FPSTR(STRING_WING), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   update();  // set defaults
}


DEM_NAMESPACE* SailorModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(SAILOR_STR_SAILOR,0,true);
}

void SailorModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
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

  /*
  Notes
   * negative cross-track means the target is to the left, positive to the right

  */

  // local shortcuts
  float h = _subs[SAILOR_SUB_HEADING_E].param.data.f[0];
  float w = _subs[SAILOR_SUB_WIND_E].param.data.f[0];
  float t = _subs[SAILOR_SUB_TARGET_E].param.data.f[0];
  float ct = _subs[SAILOR_SUB_CROSSTRACK_E].param.data.f[0];
  float c = _params[SAILOR_PARAM_COURSE_E].data.f[0];

  // -- algo 1 --
  // evaluate all potential course optionsl, using the polar for each
  // dot the polar value into the target vector
  // pick the one with the largest vector magnitude


  //uint8_t coursePolarIndex = polarIndexForAngle(h); // what region are we trying to sail in?
  //boolean rightPolar = headingPolarIndex < 16;

  // calc sheet based on delta between heading and wind
  float wing = shortestSignedDistanceBetweenCircularValues(h, w);
  float sheet = fabs(wing) / 180;
  if (sheet > 1) sheet = 1;

  if (_tackLocked && (_lastCrossTrackPositive ? ct < -1 : ct > 1)) _tackLocked = false;
  if (fabs(ct) > 1.2) {
    _tackLocked = false;
  }

  //boolean tackNeeded = rightPolar ? (ct < -1) : (ct > 1);
  //if (fabs(ct) > 3) tackNeeded =true; // catchall
  //boolean tackNeeded = (!_tackLocked) && fabs(ct) > 1;


  /*

  if abs(crosstrack > 1) then its time to tack
     when evaluating potential headings, need to rule out headings that would increase cross-track
     i.e. zero the pv values
     if crosstrack positive, then relative headings need to be between 0 and 90 degrees
     if crosstrack negative, then relative headings need to be between 0 and -90 degrees
     where relative heading is shortestSignedDistanceBetweenCircularValues(target, potential heading)

  */


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
      _params[SAILOR_PARAM_SPEED_E].data.uint8[i] = pv;
    } else {
      _params[SAILOR_PARAM_SPEED2_E].data.uint8[i-16] = pv;
    }
    if (pv > maxPV) {
      maxPV = pv;
      c = ang + w; // put back into world frame
      newTackIsStarboard = i > 15;
    }
  }

  // ensure course is in range 0.. 360
  c = fmod(c, 360);
  if (c < 0) c += 360;

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

  // remap sheet in range -1 to 1
  sheet = (sheet * 2) - 1;

  uint8_t flags[3];
  flags[0] = _starboardTack ? 1 : 0;
  flags[1] = _tackLocked ? 1 : 0;
  flags[2] = _lastCrossTrackPositive ? 1 : 0;

  updateAndPublishParam(&_params[SAILOR_PARAM_FLAGS_E], (uint8_t*)&flags, sizeof(flags));

  updateAndPublishParam(&_params[SAILOR_PARAM_SHEET_E], (uint8_t*)&sheet, sizeof(sheet));
  updateAndPublishParam(&_params[SAILOR_PARAM_COURSE_E], (uint8_t*)&c, sizeof(c));

  // rescale
  wing = wing > 0 ? 1 : -1;
  updateAndPublishParam(&_params[SAILOR_PARAM_WING_E], (uint8_t*)&wing, sizeof(wing));
}


void SailorModule::loop() {
  DroneModule::loop();
  publishParamEntry(&_params[SAILOR_PARAM_SPEED_E]);
  publishParamEntry(&_params[SAILOR_PARAM_SPEED2_E]);
  publishParamEntry(&_params[SAILOR_PARAM_WING_E]);
}
