/*

@type          Depth
@inherits      Drone
@description   Manages a simple Depth sensor (sonar transducer) controlled via Trigger and Echo pins

@guide >>>

Notes:
In the oceans the speed of sound varies between 1,450 and 1,570 metres per second or 0.145 cm per microsecond.
Given the transducer requires the sound pulse to travel to the bottom and back the effective distance per microsend is actually halved or 0.0725 cm per microsecond.
<<<

@config >>>
// New Depth sensor bound to channel 30
Depth.new 30
  name "Depth"
  status true         // enabled
  interval 1000       // sample once per second
  pins OUT0_0 OUT0_1  // tigger and echo pins
  limits 0.25 10      // set sensor range to min=0.25m, max=10m
  speed 1480          // speed of sound in fresh water = 1480m/s
  $location [@>5.8]   // subscribe to GPS location
  distance 5          // publish log entires every 5m

  .publish "depth"    // publish the measured depth
  .publish "log"      // publish a combined log entry containing GPS location and depth

  // .publish "speed"
  // .publish "pins"
  // .publish "limits"
  // .publish "distance"
.done
<<<

*/
#ifndef DEPTH_MODULE_H
#define DEPTH_MODULE_H

#include "../DroneModule.h"


// pubs

// @pub 10;u8;2;w;pins;Pin connections for the depth module (Trigger, Echo)
#define DEPTH_PARAM_PINS         10
#define DEPTH_PARAM_PINS_E       0

// @pub 11;f;1;w;speed;Set the speed of sound in water (default 1480 for fresh water)
#define DEPTH_PARAM_SPEED        11
#define DEPTH_PARAM_SPEED_E      1

// @pub 12;f;2;w;limits;Min and max depth limits in meters (Default 0.25 10)
#define DEPTH_PARAM_LIMITS       12
#define DEPTH_PARAM_LIMITS_E     2

// @pub 13;f;1;r;depth;Measured depth (or zero if unable to measure)
#define DEPTH_PARAM_DEPTH        13
#define DEPTH_PARAM_DEPTH_E      3

// @pub 14;f;3;r;log;Composite log entry combining current GPS location and depth reading into a single param
#define DEPTH_PARAM_LOG          14
#define DEPTH_PARAM_LOG_E        4

// @pub 15;f;1;w;distance;Minimum distance between log entires.  Will only publish a fresh log entry if more than distance from last sample location.  Default 0m.
#define DEPTH_PARAM_DISTANCE     15
#define DEPTH_PARAM_DISTANCE_E   5

#define DEPTH_PARAM_ENTRIES      6

// subs
// @sub 20;21;f;2;location;Current location from GPS
#define DEPTH_SUB_LOCATION           20
#define DEPTH_SUB_LOCATION_ADDR      21
#define DEPTH_SUB_LOCATION_E         0

#define DEPTH_SUBS              1

// indices to the _pins array
#define DEPTH_PIN_TRIGGER   0
#define DEPTH_PIN_ECHO      1


static const char DEPTH_STR_DEPTH[] PROGMEM = "Depth";

class DepthModule:  public DroneModule {
protected:
  float _logPos[2];  // coordinates of last log entry
public:

  DepthModule(uint8_t id, DroneSystem* ds);
  
  virtual void setup();

  void loop();

};

#endif
