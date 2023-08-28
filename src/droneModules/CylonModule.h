/*

@type Cylon
@inherits Drone
@description Manage a strip of NEOPixels (WS2812B) - originally created for the Kit speedboat

*/
#ifndef CYLON_MODULE_H
#define CYLON_MODULE_H

#include "../DroneModule.h"
#include <NeoPixelBrightnessBus.h>


//pubs
// @pub 12;u8;1;pins;Which output pin is connected to the pixel strip
#define CYLON_PARAM_PINS            12
#define CYLON_PARAM_PINS_E          0

// @pub 13;u8;1;numPixels;How many pixels in the strip
#define CYLON_PARAM_NUMPIXELS       13
#define CYLON_PARAM_NUMPIXELS_E     1

#define CYLON_PARAM_ENTRIES         2

// subs
// @sub 8;9;f;1;left;Left motor speed
#define CYLON_SUB_LEFT         8
#define CYLON_SUB_LEFT_ADDR    9
#define CYLON_SUB_LEFT_E       0

// @sub 10;11;f;1;right;Right motor speed
#define CYLON_SUB_RIGHT         10
#define CYLON_SUB_RIGHT_ADDR    11
#define CYLON_SUB_RIGHT_E       1

#define CYLON_SUBS             2


static const char CYLON_STR_CYLON[] PROGMEM = "Cylon";

class CylonModule:  public DroneModule {
protected:
  uint8_t _firstPixel;
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> *_strip;
public:

  CylonModule(uint8_t id, DroneSystem* ds);
  
  void disable();

  virtual void setup();
  virtual void loop();

  void update();

};

#endif
