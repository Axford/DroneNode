#include "NeopixelModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"


NeopixelModule::NeopixelModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(NEOPIXEL_STR_NEOPIXEL));
   _pins[0] = 0;

   _numPixels = 4;
   //_colourOrder = NEO_GRB;

   _blackout.r = 0;
   _blackout.g = 0;
   _blackout.b = 0;

   // subs
   initSubs(NEOPIXEL_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[NEOPIXEL_SUB_SCENE_E];
   sub->addrParam = NEOPIXEL_SUB_SCENE_ADDR;
   sub->param.param = NEOPIXEL_SUB_SCENE;
   setParamName(FPSTR(DRONE_STR_SCENE), &sub->param);
   sub->param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 16);

   initScene((NEOPIXEL_SCENE*)&sub->param.data.c);

   sub = &_subs[NEOPIXEL_SUB_ACTIVESCENE_E];
   sub->addrParam = NEOPIXEL_SUB_ACTIVESCENE_ADDR;
   sub->param.param = NEOPIXEL_SUB_ACTIVESCENE;
   setParamName(FPSTR(DRONE_STR_ACTIVESCENE), &sub->param);
   sub->param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   sub->param.data.uint8[0] = 0;

   // pubs
   initParams(NEOPIXEL_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   for (uint8_t i=0; i < NEOPIXEL_PARAM_ENTRIES; i++) {
     param = &_params[NEOPIXEL_PARAM_SCENE0_E + i];
     param->param = NEOPIXEL_PARAM_SCENE0 + i;
     setParamName(FPSTR(DRONE_STR_SCENE), param);
     param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 16);
     NEOPIXEL_SCENE *scene = (NEOPIXEL_SCENE*)&param->data.c;
     initScene(scene);
     scene->brightness = 50 + (i*50);
   }
}


void NeopixelModule::initScene(NEOPIXEL_SCENE *scene) {
  scene->brightness = 50;
  scene->effect = NEOPIXEL_SOLID;
  scene->p1 = 0;
  scene->p2 = 0;
  scene->segments[0].r = 255;
  scene->segments[0].g = 255;
  scene->segments[0].b = 255;

  scene->segments[1].r = 0;
  scene->segments[1].g = 255;
  scene->segments[1].b = 0;

  scene->segments[2].r = 0;
  scene->segments[2].g = 0;
  scene->segments[2].b = 255;

  scene->segments[3].r = 255;
  scene->segments[3].g = 0;
  scene->segments[3].b = 0;
}


void NeopixelModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

  DroneModule::parsePins(obj, _pins, (uint8_t)sizeof(_pins));

  _numPixels = obj[F("numPixels")] | _numPixels;
  //_colourOrder = obj[F("colourOrder")] | _colourOrder;
  // TODO: think about best way to define/load colour order

  // load scenes
  if (obj.containsKey(DRONE_STR_SCENES)) {
    Log.noticeln(DRONE_STR_SCENES);
    JsonArray array = obj[DRONE_STR_SCENES].as<JsonArray>();

    uint8_t i = 0;
    for(JsonVariant v : array) {
      if (i < NEOPIXEL_PARAM_ENTRIES) {
        // update scene
        JsonArray values = v.as<JsonArray>();
        if (values.size() <= 16) {
          for (uint8_t j=0; j<values.size(); j++)
          _params[i].data.uint8[j] = values[j] | _params[i].data.uint8[j];
        }
      }

      i++;
    }
  }

  // now instance the _strip object:
  //_strip = new Adafruit_NeoPixel(_numPixels, _pins[0], _colourOrder + NEO_KHZ800);
  //CRGB leds[NUM_LEDS]
  //_strip = new CRGB(_numPixels);
  _strip = new NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>(_numPixels, _pins[0]);
}


void NeopixelModule::disable() {
  _strip->SetBrightness(25);
  _strip->ClearTo(RgbColor(0,0,0));

  _strip->Show();
  DroneModule::disable();
}


void NeopixelModule::setup() {
  DroneModule::setup();

  if (_pins[0] > 0) {
    _strip->Begin();
    _strip->Show();
    _strip->SetBrightness(25); // to be overridden by scenes

    // init blue while booting
    for(uint8_t i=0; i<_numPixels; i++) {
      _strip->SetPixelColor(i, RgbColor(0, 0, 255));
    }
    _strip->Show();

  } else {
    Log.errorln(F("Undefined pin %d"), _pins[0]);
    disable();
  }
}


void NeopixelModule::loop() {
  DroneModule::loop();

  unsigned long loopTime = millis();

  if (_subs[NEOPIXEL_SUB_ACTIVESCENE_E].param.data.uint8[0] < NEOPIXEL_PARAM_ENTRIES) {
    uint8_t sceneIndex = NEOPIXEL_PARAM_SCENE0_E + _subs[NEOPIXEL_SUB_ACTIVESCENE_E].param.data.uint8[0];

    updateAndPublishParam(&_subs[NEOPIXEL_SUB_SCENE_E].param, _params[sceneIndex].data.uint8, 16);
  }

  NEOPIXEL_SCENE *scene = (NEOPIXEL_SCENE*)&_subs[NEOPIXEL_SUB_SCENE_E].param.data.c;

  if (_strip) {
    uint8_t pixPerSeg = _numPixels / NEOPIXEL_NUM_SEGMENTS;

    NEOPIXEL_COLOUR segs[NEOPIXEL_NUM_SEGMENTS];


    switch(scene->effect) {
      case NEOPIXEL_SOLID:
        for(uint8_t i=0; i<NEOPIXEL_NUM_SEGMENTS; i++) {
          segs[i] = scene->segments[i];
        }
        break;

      case NEOPIXEL_FLASH:
        boolean flash = true;
        if (scene->p1 > 0) {
          int flashTime = loopTime % (scene->p1 * 100);
          flash = (flashTime < scene->p2*100);
        }

        for(uint8_t i=0; i<NEOPIXEL_NUM_SEGMENTS; i++) {
          if (flash) {
            segs[i] = scene->segments[i];
          } else {
            segs[i] = _blackout;
          }
        }
        break;
    }

    // update segments
    _strip->SetBrightness(scene->brightness);
    for(uint8_t i=0; i<_numPixels; i++) {
      uint8_t seg = i / pixPerSeg;


      _strip->SetPixelColor(i, RgbColor(
        segs[seg].r,
        segs[seg].g,
        segs[seg].b
      ));

    }

    _strip->Show();
  }

  _effectTime = loopTime;
}


void NeopixelModule::update() {
  //
}
