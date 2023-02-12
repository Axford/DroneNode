/*
@type          Polar
@inherits      Drone
@description   Sail polar calibration module

@guide >>>
<p>Sail on a variety of headings relative to the wind, measuring speed over ground
relative to wind speed and aggregating results to build a polar plot of saiiing performance vs heading.</p>

<p>When in passthrough mode, will simply pass the Sailor heading onto its output (module chaining).  When active, will bypass the Sailor module and generate target headings.</p>

<p>In either mode, the Sailor module is left to generate the sheet command based on
relative wind direction.  This may need a separate calibration process.</p>


<p>Algorithm:</p>
<ul>
 <li>if outside the inner threshold - turn onto a heading that will orbit the target in a clockwise direction (configurable?) at distance of mid threshold, once speed over ground is over threshold, then select a new heading that goes from current location through the center of the target.
 <li>when crossing the inner threshold heading inward - record the start location and time.
 <li>when crossing the inner threshold heading outward - calculate effective heading and if close enough to target heading, then record the end location and time, compute the average speed and add to polar info.

 <li>if pass the outer threshold heading out, abort and switch to passthrough mode to allow
 nav/sailor modules to take over
 <li>if pass the outer threshold heading inward, switch to active mode
</ul>
<<<

@config >>>
[Polar=13]
  name="Polar"
  interval=1000
  $location=@>50.9
  $wind=@>50.10
  windSpeed =1
  $heading=@>Sailor.course
  target =-1.7454870, 51.5418469
  threshold =-1, 100
  radius =10, 20, 50
  mode= 1
  publish =l ocation, SOG, wind, windSpeed, heading, polar
  publish = adjHeading, mode, target, threshold, radius, samples
<<<
*/

#ifndef POLAR_MODULE_H
#define POLAR_MODULE_H

#include "../DroneModule.h"


/* pubs

Input parameters:
 - Center of target region plus Inner, mid and outer threshold radii
 - Mode - passthrough, active, reset
 - Min SOG for tack and Acceptable deviation from target heading

Output parameters:
- Polar plot (after processing)
- Polar num samples per bin
- Adjusted heading

*/

// @pub 8;f;5;target;Lon/Lat of target
#define POLAR_PARAM_TARGET         8
#define POLAR_PARAM_TARGET_E       0

// @pub 9;u8;1;mode;Mode - passthrough (default), active, reset
#define POLAR_PARAM_MODE           9
#define POLAR_PARAM_MODE_E         1

// @pub 10;f;2;threshold;Min SOG for tack and acceptable deviation from target heading
#define POLAR_PARAM_THRESHOLD      10
#define POLAR_PARAM_THRESHOLD_E    2

// @pub 11;u8;16;polar;Calculated polar plot
#define POLAR_PARAM_POLAR          11
#define POLAR_PARAM_POLAR_E        3

// @pub 12;u8;16;samples;Number of samples per polar bin
#define POLAR_PARAM_SAMPLES        12
#define POLAR_PARAM_SAMPLES_E      4

// @pub 13;f;3;radius;Inner, mid and outer target radii
#define POLAR_PARAM_RADIUS         13
#define POLAR_PARAM_RADIUS_E       5

// @pub 14;f;2;adjHeading;Adjusted heading - either passthrough of <b>Heading</b> or generated heading depending on mode
#define POLAR_PARAM_ADJ_HEADING    14
#define POLAR_PARAM_ADJ_HEADING_E  6

#define POLAR_PARAM_ENTRIES        7

/*
Subs
- Wind speed
- Wind angle
- GPS location
- GPS speed over ground
- Heading from Sailor
*/

// @sub 20;21;f;2;location;Current location from GPS
#define POLAR_SUB_LOCATION           30
#define POLAR_SUB_LOCATION_ADDR      31
#define POLAR_SUB_LOCATION_E         0

// @sub 32;33;f;1;SOG;Speed over ground from GPS
#define POLAR_SUB_SOG                32
#define POLAR_SUB_SOG_ADDR           33
#define POLAR_SUB_SOG_E              1

// @sub 34;35;f;1;wind;Wind angle
#define POLAR_SUB_WIND               34
#define POLAR_SUB_WIND_ADDR          35
#define POLAR_SUB_WIND_E             2

// @sub 36;37;f;1;windSpeed;Wind speed
#define POLAR_SUB_WIND_SPEED         36
#define POLAR_SUB_WIND_SPEED_ADDR    37
#define POLAR_SUB_WIND_SPEED_E       3

// @sub 38;39;f;1;heading;Target heading from Sailor
#define POLAR_SUB_HEADING            38
#define POLAR_SUB_HEADING_ADDR       39
#define POLAR_SUB_HEADING_E          4

#define POLAR_SUBS                   5


enum {
  POLAR_MODE_PASSTHROUGH,
  POLAR_MODE_ACTIVE,
  POLAR_MODE_RESET
} POLAR_MODE;

enum {
  POLAR_REGION_OUT,
  POLAR_REGION_MID,
  POLAR_REGION_IN
} POLAR_REGION;

static const char POLAR_STR_POLAR[] PROGMEM = "Polar";

class PolarModule:  public DroneModule {
protected:
  float _polarVals[16]; // aggregated values per bin
  float _startPos[2];   // where did we start this run
  unsigned long _startTime;  // when we started the run
  uint8_t  _region;
public:

  PolarModule(uint8_t id, DroneSystem* ds);

  uint8_t polarIndexForAngle(float ang);

  void updatePolar();  // recalc from samples

  void setup();
  void update();
  void loop();

  void loopActive();
  void loopReset();

};

#endif
