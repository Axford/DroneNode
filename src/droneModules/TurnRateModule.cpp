#include "TurnRateModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

TurnRateModule::TurnRateModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(TURN_RATE_STR_TURN_RATE));

   _lastUpdate = 0;
   _iError = 0;
   _dError = 0;
   _lastError = 0;

   // mgmt
   _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, sizeof(TURN_RATE_STR_TURN_RATE));
   strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, TURN_RATE_STR_TURN_RATE, sizeof(TURN_RATE_STR_TURN_RATE));


   // subs
   initSubs(TURN_RATE_SUBS);

   _subs[TURN_RATE_SUB_TARGET_E].addrParam = TURN_RATE_SUB_TARGET_ADDR;
   _subs[TURN_RATE_SUB_TARGET_E].param.param = TURN_RATE_SUB_TARGET;
   _subs[TURN_RATE_SUB_TARGET_E].param.name = FPSTR(STRING_TARGET);
   _subs[TURN_RATE_SUB_TARGET_E].param.nameLen = sizeof(STRING_TARGET);

   _subs[TURN_RATE_SUB_HEADING_E].addrParam = TURN_RATE_SUB_HEADING_ADDR;
   _subs[TURN_RATE_SUB_HEADING_E].param.param = TURN_RATE_SUB_HEADING;
   _subs[TURN_RATE_SUB_HEADING_E].param.name = FPSTR(STRING_HEADING);
   _subs[TURN_RATE_SUB_HEADING_E].param.nameLen = sizeof(STRING_HEADING);

   _subs[TURN_RATE_SUB_PID_E].addrParam = TURN_RATE_SUB_PID_ADDR;
   _subs[TURN_RATE_SUB_PID_E].param.param = TURN_RATE_SUB_PID;
   _subs[TURN_RATE_SUB_PID_E].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _subs[TURN_RATE_SUB_PID_E].param.name = FPSTR(STRING_PID);
   _subs[TURN_RATE_SUB_PID_E].param.nameLen = sizeof(STRING_PID);


   // outputs
   initParams(TURN_RATE_PARAM_ENTRIES);

   _params[TURN_RATE_PARAM_TURN_RATE_E].param = TURN_RATE_PARAM_TURN_RATE;
   _params[TURN_RATE_PARAM_TURN_RATE_E].name = FPSTR(STRING_TURN_RATE);
   _params[TURN_RATE_PARAM_TURN_RATE_E].nameLen = sizeof(STRING_TURN_RATE);
   _params[TURN_RATE_PARAM_TURN_RATE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
}


DEM_NAMESPACE* TurnRateModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(TURN_RATE_STR_TURN_RATE,0,true);
}

void TurnRateModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_TARGET, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$target"), DRONE_LINK_MSG_TYPE_FLOAT, pha);

  dem->registerCommand(ns, STRING_HEADING, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$heading"), DRONE_LINK_MSG_TYPE_FLOAT, pha);

  dem->registerCommand(ns, STRING_PID, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$PID"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
}


float TurnRateModule::getRotationDistance(float origin, float target){
  float signedDiff = 0.0;
  float raw_diff = origin > target ? origin - target : target - origin;
  float mod_diff = fmod(raw_diff, 360); //equates rollover values. E.g 0 == 360 degrees in circle

  if(mod_diff > (360/2) ){
    //There is a shorter path in opposite direction
    signedDiff = (360 - mod_diff);
    if(target>origin) signedDiff = signedDiff * -1;
  } else {
    signedDiff = mod_diff;
    if(origin>target) signedDiff = signedDiff * -1;
  }

  return signedDiff;
}


void TurnRateModule::update() {
  if (!_setupDone) return;

  unsigned long updateTime = millis();
  float dt = (updateTime - _lastUpdate) / 1000.0;


  // calc and publish new speeds

  // check we've received valid heading and target
  if (!_subs[TURN_RATE_SUB_HEADING_E].received ||
      !_subs[TURN_RATE_SUB_TARGET_E].received) return;

  // local shortcuts
  float h = _subs[TURN_RATE_SUB_HEADING_E].param.data.f[0];
  float t = _subs[TURN_RATE_SUB_TARGET_E].param.data.f[0];

  // calc shortest signed distance
  // positive values indicate a clockwise turn
  float err = getRotationDistance( h, t );

  // limit to 90 for ease
  if (err > 90) err = 90;
  if (err < -90) err = -90;

  // update I and D terms
  _iError += err * dt;
  _dError = (err - _lastError) / dt;

  // apply PID cooefficients
  float tr =
    err * _subs[TURN_RATE_SUB_PID_E].param.data.f[0] +
    _iError * _subs[TURN_RATE_SUB_PID_E].param.data.f[1] +
    _dError * _subs[TURN_RATE_SUB_PID_E].param.data.f[2];

  _lastError = err;

  // apply limits
  if (tr > 10) tr = 10;
  if (tr < -10) tr = -10;

  updateAndPublishParam(&_params[TURN_RATE_PARAM_TURN_RATE_E], (uint8_t*)&tr, sizeof(tr));
}
