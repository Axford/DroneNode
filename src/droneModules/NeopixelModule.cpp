#include "NeopixelModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

NeopixelModule::NeopixelModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs)
 {
   setTypeName(FPSTR(NEOPIXEL_STR_NEOPIXEL));
   _strip = NULL;
   //_pins[0] = 0;

   //_numPixels = 4;
   //_colourOrder = NEO_GRB;

   _blackout.r = 0;
   _blackout.g = 0;
   _blackout.b = 0;

   // subs
   initSubs(NEOPIXEL_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[NEOPIXEL_SUB_SCENE_E];
   sub->addrParam = NEOPIXEL_SUB_SCENE_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NEOPIXEL_SUB_SCENE);
   setParamName(FPSTR(STRING_SCENE), &sub->param);
   sub->param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 16);

   initScene((NEOPIXEL_SCENE*)&sub->param.data.c);

   // pubs
   initParams(NEOPIXEL_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[NEOPIXEL_PARAM_PINS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NEOPIXEL_PARAM_PINS);
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);

   param = &_params[NEOPIXEL_PARAM_NUMPIXELS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, NEOPIXEL_PARAM_NUMPIXELS);
   setParamName(FPSTR(STRING_NUMPIXELS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[NEOPIXEL_PARAM_NUMPIXELS_E].data.uint8[0] = 4;
}


DEM_NAMESPACE* NeopixelModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(NEOPIXEL_STR_NEOPIXEL,0,true);
}

void NeopixelModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_PINS, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_NUMPIXELS, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_SCENE, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, PSTR("$scene"), DRONE_LINK_MSG_TYPE_UINT8_T, pha);
}


void NeopixelModule::initScene(NEOPIXEL_SCENE *scene) {
  scene->brightness = 10;
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


void NeopixelModule::disable() {
  Log.noticeln(F("[NM.d]"));
  if (_strip) {
    _strip->SetBrightness(25);
    _strip->ClearTo(RgbColor(0,0,0));

    _strip->Show();
  }
  DroneModule::disable();
  Log.noticeln(F("[NM.d] end"));
}


void NeopixelModule::setup() {
  //Log.noticeln(F("[NM.s]"));
  DroneModule::setup();
  if (_params[NEOPIXEL_PARAM_PINS_E].data.uint8[0] > 0) {
    if (_strip == NULL) {
      _strip = new NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>(_params[NEOPIXEL_PARAM_NUMPIXELS_E].data.uint8[0], _params[NEOPIXEL_PARAM_PINS_E].data.uint8[0]);
    }

    _strip->Begin();
    _strip->Show();
    _strip->SetBrightness(25); // to be overridden by scenes


    // init blue while booting
    for(uint8_t i=0; i<_params[NEOPIXEL_PARAM_NUMPIXELS_E].data.uint8[0]; i++) {
      _strip->SetPixelColor(i, RgbColor(0, 0, 255));
    }
    _strip->Show();

  } else {
    Log.errorln(F("Undefined pin %d"), _params[NEOPIXEL_PARAM_PINS_E].data.uint8[0]);
    disable();
  }
}


void NeopixelModule::loop() {
  DroneModule::loop();
  //Log.noticeln(F("[NM.l]"));

  unsigned long loopTime = millis();

  NEOPIXEL_SCENE *scene = (NEOPIXEL_SCENE*)&_subs[NEOPIXEL_SUB_SCENE_E].param.data.c;

  if (_strip) {
    uint8_t pixPerSeg = _params[NEOPIXEL_PARAM_NUMPIXELS_E].data.uint8[0] / NEOPIXEL_NUM_SEGMENTS;

    NEOPIXEL_COLOUR segs[NEOPIXEL_NUM_SEGMENTS];


    boolean doFlash = false;
    float b = 255;

    switch(scene->effect) {
      case NEOPIXEL_SOLID:
        _strip->SetBrightness(scene->brightness);
        for(uint8_t i=0; i<NEOPIXEL_NUM_SEGMENTS; i++) {
          segs[i] = scene->segments[i];
        }
        break;

      case NEOPIXEL_FLASH:
        doFlash = true;
        _strip->SetBrightness(scene->brightness);
        if (scene->p1 > 0) {
          int flashTime = loopTime % (scene->p1 * 100);
          doFlash = (flashTime < scene->p2*100);
        }

        for(uint8_t i=0; i<NEOPIXEL_NUM_SEGMENTS; i++) {
          if (doFlash) {
            segs[i] = scene->segments[i];
          } else {
            segs[i] = _blackout;
          }
        }
        break;

      case NEOPIXEL_PULSE:

        if (scene->p1 > 0) {
          b = 0.5 + sin( 2.0 * PI * (float)(loopTime % (scene->p1 * 100)) / (scene->p1 * 100) )/2.0;
          b *= scene->brightness;
        }

        // set brightness to pulse (ensure in range)
        if (b > 255) b = 255;
        if (b < 0) b = 0;
        _strip->SetBrightness(b);

        for(uint8_t i=0; i<NEOPIXEL_NUM_SEGMENTS; i++) {
          segs[i] = scene->segments[i];
        }
        break;
    }

    // update segments
    for(uint8_t i=0; i<_params[NEOPIXEL_PARAM_NUMPIXELS_E].data.uint8[0]; i++) {
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
  if (!_setupDone) return;
}
