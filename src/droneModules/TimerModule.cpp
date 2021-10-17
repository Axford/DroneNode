#include "TimerModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"


TimerModule::TimerModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem):
  DroneModule ( id, dmm, dlm, dem ),
  _lastTime(0),
  _interval(1000)
 {
   setTypeName(FPSTR(TIMER_STR_TIMER));
   _msg.channel(0);
   _msg.param(0);

   _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, sizeof(TIMER_STR_TIMER));
   strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, TIMER_STR_TIMER, sizeof(TIMER_STR_TIMER));
}

void TimerModule::handleLinkMessage(DroneLinkMsg *msg) {
  DroneModule::handleLinkMessage(msg);
}

void TimerModule::setup() {
  Log.noticeln("Timer setup complete");
}

void TimerModule::loop() {
  if (!_enabled) return;

  uint32_t t = millis();
  if (t > _lastTime + _interval) {
    Log.noticeln("Timer triggered: %d", _id);
    _dlm->publish(_msg);
    _lastTime = t;
  }
}
