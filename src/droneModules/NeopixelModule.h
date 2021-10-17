/*

Neopixel Modules

Manage a strip of NEOPixels (WS2812B)

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
#define NEOPIXEL_PARAM_SCENE0        100
#define NEOPIXEL_PARAM_SCENE0_E      0

#define NEOPIXEL_PARAM_ENTRIES       4  // number of scenes to generate


// subs
#define NEOPIXEL_SUB_SCENE         8
#define NEOPIXEL_SUB_SCENE_ADDR    9
#define NEOPIXEL_SUB_SCENE_E       0

#define NEOPIXEL_SUB_ACTIVESCENE         10
#define NEOPIXEL_SUB_ACTIVESCENE_ADDR    11
#define NEOPIXEL_SUB_ACTIVESCENE_E       1

#define NEOPIXEL_SUBS                    2


static const char NEOPIXEL_STR_NEOPIXEL[] PROGMEM = "Neopixel";

class NeopixelModule:  public DroneModule {
protected:
  uint8_t _pins[1];
  uint8_t _numPixels;
  uint8_t _colourOrder;
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> *_strip;
  unsigned long _effectTime;
  NEOPIXEL_COLOUR _blackout;
public:

  NeopixelModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);

  void initScene(NEOPIXEL_SCENE *scene);

  void loadConfiguration(JsonObject &obj);

  void disable();

  virtual void setup();
  virtual void loop();

  void update();

};

#endif
