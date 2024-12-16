/*

@type         Neopixel
@inherits     Drone
@category     Output.LED
@description  Manage a strip of NEOPixels (WS2812B)

TODO - rework alongside DroneLED to use the v4 LED header

* Disable module will clear LEDs first (turn off) -so can be used to toggle on/off
* Base settings:
  - Pin
  - Num pixels (automatically divided into four segments)
  - Colour order hard cided to GRB for now (e.g. GRB vs RGB)
  - Driver hard coded to NEO_KHZ800, as most common
* Subs
  - Scene number (0..x) - so other modules can select a scene
  - Active scene - so other modules can configure the active scene - for extensibility
* Params
  - 0..x scenes - with addresses incrementing from 100 (like waypoints)

* Scene (struct stored in uint8_t msg):
  - brightness - overall brightness for this scene
  - effect:
      0: solid Colour
      1: flash
      2: theatre chase
      3: pulse
  - effect param 1:
      e.g. flash speed
  - effect param 2:
      e.g. flash duty cycle
  - segment 1..4 - r,g,b (12 bytes total)

*/
#ifndef NEOPIXEL_MODULE_H
#define NEOPIXEL_MODULE_H

#include "../DroneModule.h"
//#include <Adafruit_Neopixel.h>
#include <NeoPixelBrightnessBus.h>

#define NEOPIXEL_NUM_SEGMENTS    4

enum NEOPIXEL_EFFECTS {
  NEOPIXEL_SOLID,
  NEOPIXEL_FLASH,
  NEOPIXEL_PULSE,
  NEOPIXEL_CHASE
};

struct NEOPIXEL_COLOUR {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} __packed;

struct NEOPIXEL_SCENE {
  uint8_t brightness;
  uint8_t effect;
  uint8_t p1;
  uint8_t p2;
  NEOPIXEL_COLOUR segments[NEOPIXEL_NUM_SEGMENTS];
} __packed;

//pubs
// @pub 12;u8;1;w;pins;Output pin for Neopixel strip
#define NEOPIXEL_PARAM_PINS          12
// @pub 13;u8;1;w;numPixels;Number of pixels in strip
#define NEOPIXEL_PARAM_NUMPIXELS     13

#define NEOPIXEL_PARAM_PINS_E          0
#define NEOPIXEL_PARAM_NUMPIXELS_E     1

#define NEOPIXEL_PARAM_ENTRIES       2

// subs
// @sub 8;9;u8;16;scene;Pixel components and effects for the scene
#define NEOPIXEL_SUB_SCENE         8
#define NEOPIXEL_SUB_SCENE_ADDR    9
#define NEOPIXEL_SUB_SCENE_E       0

#define NEOPIXEL_SUBS                    1


static const char NEOPIXEL_STR_NEOPIXEL[] PROGMEM = "Neopixel";

class NeopixelModule:  public DroneModule {
protected:
  //uint8_t _pins[1];
  //uint8_t _numPixels;
  uint8_t _colourOrder;
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> *_strip;
  unsigned long _effectTime;
  NEOPIXEL_COLOUR _blackout;
public:

  NeopixelModule(uint8_t id, DroneSystem* ds);

  void initScene(NEOPIXEL_SCENE *scene);

  void disable();

  virtual void setup();
  virtual void loop();

  void update();

};

#endif
