#include "PanSerialModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"
#include "../pinConfig.h"

PanSerialModule::PanSerialModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(PAN_SERIAL_STR_PAN_SERIAL));

   _lastUpdate = 0;
   _iError = 0;
   _dError = 0;
   _lastError = 0;
   _lastTarget = 0;

   // set default interval to 1000
   // @default interval = 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   // subs
   initSubs(PAN_SERIAL_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[PAN_SERIAL_SUB_TARGET_E];
   sub->addrParam = PAN_SERIAL_SUB_TARGET_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PAN_SERIAL_SUB_TARGET);
   setParamName(FPSTR(STRING_TARGET), &sub->param);

   sub = &_subs[PAN_SERIAL_SUB_HEADING_E];
   sub->addrParam = PAN_SERIAL_SUB_HEADING_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PAN_SERIAL_SUB_HEADING);
   setParamName(FPSTR(STRING_HEADING), &sub->param);


   // pubs
   initParams(PAN_SERIAL_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[PAN_SERIAL_PARAM_PID_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, PAN_SERIAL_PARAM_PID);
   setParamName(FPSTR(STRING_PID), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[PAN_SERIAL_PARAM_PID_E].data.f[0] = 0.005;
   _params[PAN_SERIAL_PARAM_PID_E].data.f[1] = 0;
   _params[PAN_SERIAL_PARAM_PID_E].data.f[2] = 0.0001;

   param = &_params[PAN_SERIAL_PARAM_PAN_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, PAN_SERIAL_PARAM_PAN);
   setParamName(FPSTR(STRING_PAN), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[PAN_SERIAL_PARAM_PAN_E].data.uint8[0] = 0;
}


float PanSerialModule::getRotationDistance(float origin, float target){
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


void PanSerialModule::setup() {
  DroneModule::setup();

  Serial2.begin(9600, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX);
  _port = &Serial2;
}


void PanSerialModule::update() {
  if (!_setupDone) return;

  unsigned long updateTime = millis();
  float dt = (updateTime - _lastUpdate) / 1000.0;

  // check we've received valid heading and target
  /*
  if (!_subs[PAN_SERIAL_SUB_HEADING_E].received ||
      !_subs[PAN_SERIAL_SUB_TARGET_E].received) return;
      */

  // local shortcuts
  float h = _subs[PAN_SERIAL_SUB_HEADING_E].param.data.f[0];  // current heading
  float t = _subs[PAN_SERIAL_SUB_TARGET_E].param.data.f[0];   // target heading

  // calc shortest signed distance
  // positive values indicate a clockwise turn
  float panTarget = t - h;

  // keep in range 0..360
  if (panTarget < 0) panTarget += 360;
  if (panTarget > 360) panTarget -= 360;


  // moving average
  //panTarget = (_lastTarget * 7 + panTarget) / 8;

  _lastTarget = panTarget;

  // limit range (again, just to be sure)
  panTarget = constrain(panTarget, 0, 360);

  // map to 0..255
  panTarget = panTarget * 255 / 360;

  updateAndPublishParam(&_params[PAN_SERIAL_PARAM_PAN_E], (uint8_t*)&panTarget, sizeof(panTarget));

  // send over serial
  _port->write('^');
  _port->write((uint8_t)panTarget);
  _port->write('!');
}
