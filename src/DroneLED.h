/*

Manage status LED(s) - either built-on or Neopixel string

Export an interface for other modules to set neopixel colours

*/

#ifndef DRONE_LED_H
#define DRONE_LED_H


#include "Arduino.h"
#include "pinConfig.h"
#include <NeoPixelBrightnessBus.h>

#define DRONE_LED_MAX_PIXELS    16+1  // 32 + 1

#define DRONE_LED_PIN  PIN_LED

#define DRONE_LED_HW_BUILTIN    0
#define DRONE_LED_HW_NEOPIXEL   1


/*

Status LED states / colours

white = starting up
red = error
blue = running, no wifi
green = running, wifi

*/

#define DRONE_LED_STATE_STARTUP           0
#define DRONE_LED_STATE_ERROR             1
#define DRONE_LED_STATE_RUNNING_NO_WIFI   2
#define DRONE_LED_STATE_RUNNING_WIFI      3


// forward decl
class DroneSystem;


//--------------------------------------------------------
// DroneLED
//--------------------------------------------------------
class DroneLED {
protected:
  DroneSystem* _ds;
  uint8_t _hw;
  uint8_t _state;

  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> *_strip;

public:
  DroneLED(DroneSystem* ds);

  void setState(uint8_t newState);

  void update();

  void loop();
};


#endif
