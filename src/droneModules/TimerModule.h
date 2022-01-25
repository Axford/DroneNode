/*

@type Timer
@description Emits a configured message on a set interval (WIP)

// TODO - rework!!!
*/
#ifndef TIMER_MODULE_H
#define TIMER_MODULE_H

#include "../DroneModule.h"
#include "../DroneLinkMsg.h"


static const char TIMER_STR_TIMER[] PROGMEM = "Timer";

class TimerModule:  public DroneModule {
protected:
  uint32_t _lastTime;
  uint32_t _interval;
public:
  DroneLinkMsg _msg;

  TimerModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  virtual void handleLinkMessage(DroneLinkMsg *msg);

  virtual void setup();
  virtual void loop();

  void setInterval(uint32_t t) { _interval = t; }

};

#endif
