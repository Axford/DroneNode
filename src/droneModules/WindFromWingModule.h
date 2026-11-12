/*
@type          WindFromWing
@inherits      Drone
@category      Logic
@description   Estimate Wind Direction from Wing Compass and Tail position

@guide >>>
<p>Subscribe the Sailor module to the Wind output parameter, instead of a regular wind vane, to use the Wing as an estimator of the Wind angle.</p>
<<<

@config >>>
[ WindFromWing = 12 ]
  name = WindFromWing
  AOA = 15
  $wing = @>Sailor.wing
  $heading = 64>10.22
  publish = wing, heading, AOA, wind
<<<

*/

#ifndef WIND_FROM_WING_MODULE_H
#define WIND_FROM_WING_MODULE_H

#include "../DroneModule.h"

// subs
// @sub 10;11;f;1;wing;Wing tail command angle from Sailor.wing
#define WIND_FROM_WING_SUB_WING            10
#define WIND_FROM_WING_SUB_WING_ADDR       11
#define WIND_FROM_WING_SUB_WING_E          0

// @sub 12;13;f;1;heading;Current heading of the Wing from the Wing's Compass.heading
#define WIND_FROM_WING_SUB_HEADING         12
#define WIND_FROM_WING_SUB_HEADING_ADDR    13
#define WIND_FROM_WING_SUB_HEADING_E       1

#define WIND_FROM_WING_SUBS                2

// pubs
// @pub 14;f;1;w;AOA;AOA obtained from a Wing value of 1... i.e. multiple Wing command by AOA value to get actual AOA
#define WIND_FROM_WING_PARAM_AOA           14
#define WIND_FROM_WING_PARAM_AOA_E         0

// @pub 15;f;1;r;wind;Estimated Wind angle (world coordinates)
#define WIND_FROM_WING_PARAM_WIND          15
#define WIND_FROM_WING_PARAM_WIND_E        1


// @pub 16;u8;1;w;samples;Sample depth for moving average, 1 per interval (default 10)
#define WIND_FROM_WING_PARAM_SAMPLES       16
#define WIND_FROM_WING_PARAM_SAMPLES_E     2


#define WIND_FROM_WING_PARAM_ENTRIES       3



static const char WIND_FROM_WING_STR_WIND_FROM_WING[] PROGMEM = "WindFromWing";

class WindFromWingModule:  public DroneModule {
protected:
  uint8_t _dirSample;  // how many wind samples have we accumulated

public:

  WindFromWingModule(uint8_t id, DroneSystem* ds);

  void loop();
};

#endif
