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
#define DRONE_LED_STATE_UPDATING          4
#define DRONE_LED_STATE_RESTART           5


// forward decl
class DroneSystem;


//--------------------------------------------------------
// DroneLED
//--------------------------------------------------------
class DroneLED {
protected:
  uint32_t _animationTimer;
  uint8_t _animationState;
  DroneSystem* _ds;
  uint8_t _hw;
  uint8_t _state;

public:
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> *_strip;

  DroneLED(DroneSystem* ds);

  boolean isNeopixel();

  void setState(uint8_t newState);

  void update();

  void loop();
};


#endif
