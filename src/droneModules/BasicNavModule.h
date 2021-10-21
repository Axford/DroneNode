/*

Basic navigation:
   * Define a target location (lat,lon)
   * Sub to current location
   * Generate a target heading and est distance

*/
#ifndef BASIC_NAV_MODULE_H
#define BASIC_NAV_MODULE_H

#include "../DroneModule.h"


#define BASIC_NAV_PARAM_TARGET         8  // target location
#define BASIC_NAV_PARAM_TARGET_E       0

#define BASIC_NAV_PARAM_LOCATION       9  // current location
#define BASIC_NAV_PARAM_LOCATION_E       1

#define BASIC_NAV_PARAM_HEADING        10  // required heading
#define BASIC_NAV_PARAM_HEADING_E      2

#define BASIC_NAV_PARAM_DISTANCE       11  // distance to go
#define BASIC_NAV_PARAM_DISTANCE_E     3

#define BASIC_NAV_PARAM_ENTRIES        4


static const char BASIC_NAV_STR_BASIC_NAV[] PROGMEM = "BasicNav";

class BasicNavModule:  public DroneModule {
protected:
  DRONE_LINK_ADDR _target;   // target location
  DRONE_LINK_ADDR _location;  // current location


public:

  BasicNavModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  void onParamWrite(DRONE_PARAM_ENTRY *param);

  void handleLinkMessage(DroneLinkMsg *msg);

  void setup();

  void update();
};

#endif
