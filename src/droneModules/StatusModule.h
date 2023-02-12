/*

@type          Status
@inherits      Drone
@description   Monitors up to four parameters and publishes a corresponding status scene to a Neopixel module


*/
#ifndef STATUS_MODULE_H
#define STATUS_MODULE_H

#include "../DroneModule.h"


// pubs

// @pub 8;u8;16;scene;Status scene for a Neopixel module to subscribe to.  Set an initial value for scene to configure brightness, etc
#define STATUS_PARAM_SCENE         8
#define STATUS_PARAM_SCENE_E       0

// @pub 10;f;1;value1;Threshold for sub1
#define STATUS_PARAM_VALUE1        10
#define STATUS_PARAM_VALUE1_E      1

// @pub 11;f;1;value2;Threshold for sub2
#define STATUS_PARAM_VALUE2        11
#define STATUS_PARAM_VALUE2_E      2

// @pub 12;f;1;value3;Threshold for sub3
#define STATUS_PARAM_VALUE3        12
#define STATUS_PARAM_VALUE3_E      3

// @pub 13;f;1;value4;Threshold for sub4
#define STATUS_PARAM_VALUE4        13
#define STATUS_PARAM_VALUE4_E      4

#define STATUS_PARAM_ENTRIES       5

// subs
// @sub 20;21;f;2;sub1;First parameter to monitor for status
#define STATUS_SUB_SUB1            20
#define STATUS_SUB_SUB1_ADDR       21
#define STATUS_SUB_SUB1_E          0

// @sub 22;23;f;2;sub2;First parameter to monitor for status
#define STATUS_SUB_SUB2            22
#define STATUS_SUB_SUB2_ADDR       23
#define STATUS_SUB_SUB2_E          1

// @sub 24;25;f;2;sub3;First parameter to monitor for status
#define STATUS_SUB_SUB3            24
#define STATUS_SUB_SUB3_ADDR       25
#define STATUS_SUB_SUB3_E          2

// @sub 26;27;f;2;sub4;First parameter to monitor for status
#define STATUS_SUB_SUB4            26
#define STATUS_SUB_SUB4_ADDR       27
#define STATUS_SUB_SUB4_E          3

#define STATUS_SUBS                4


static const char STATUS_STR_STATUS[] PROGMEM = "Status";

class StatusModule:  public DroneModule {
protected:

public:

  StatusModule(uint8_t id, DroneSystem* ds);

  virtual void setup();

  uint8_t checkThreshold(uint8_t index);

  void loop();

};

#endif
