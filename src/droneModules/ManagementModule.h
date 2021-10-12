/*

Provides overall system management and a pub/sub interface to the DroneModuleManager

*/
#ifndef MANAGEMENT_MODULE_H
#define MANAGEMENT_MODULE_H

#include "../DroneModule.h"

/*
TODO:


*/

// published params
#define MANAGEMENT_PARAM_HOSTNAME      8
#define MANAGEMENT_PARAM_BUILD         9   // build timestamp
#define MANAGEMENT_PARAM_RESET         10  // set to 1 to trigger reset
#define MANAGEMENT_PARAM_HEAP          11
#define MANAGEMENT_PARAM_IP            12
#define MANAGEMENT_PARAM_UPTIME        13
#define MANAGEMENT_PARAM_PUBLISHRATE   14 // rate of messages published per second
#define MANAGEMENT_PARAM_CHOKED        15 // number of times a channel queue has choked (rejected a msg becuse full)
#define MANAGEMENT_PARAM_DISCOVERY     16 // enable/disable node discovery

#define MANAGEMENT_PARAM_HOSTNAME_E     0
#define MANAGEMENT_PARAM_BUILD_E        1
#define MANAGEMENT_PARAM_RESET_E        2
#define MANAGEMENT_PARAM_HEAP_E         3
#define MANAGEMENT_PARAM_IP_E           4
#define MANAGEMENT_PARAM_UPTIME_E       5
#define MANAGEMENT_PARAM_PUBLISHRATE_E  6
#define MANAGEMENT_PARAM_CHOKED_E       7
#define MANAGEMENT_PARAM_DISCOVERY_E    8

#define MANAGEMENT_PARAM_ENTRIES        9


// strings
static const char MANAGEMENT_STR_MANAGEMENT[] PROGMEM = "Management";

// class
class ManagementModule:  public DroneModule {
protected:
  unsigned long _lastRate;
public:

  ManagementModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm);

  void onParamWrite(DRONE_PARAM_ENTRY *param);

  virtual void setup();
  virtual void loop();

};

#endif
