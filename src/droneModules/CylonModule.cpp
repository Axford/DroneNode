#include "CylonModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "DroneSystem.h"

// @type Cylon

CylonModule::CylonModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(CYLON_STR_CYLON));
   _strip = NULL;
   _firstPixel = 0;

   // subs
   initSubs(CYLON_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[CYLON_SUB_LEFT_E];
   sub->addrParam = CYLON_SUB_LEFT_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, CYLON_SUB_LEFT);
   setParamName(FPSTR(STRING_LEFT), &sub->param);
   sub->param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   sub = &_subs[CYLON_SUB_RIGHT_E];
   sub->addrParam = CYLON_SUB_RIGHT_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, CYLON_SUB_RIGHT);
   setParamName(FPSTR(STRING_RIGHT), &sub->param);
   sub->param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);


   // pubs
   initParams(CYLON_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[CYLON_PARAM_PINS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, CYLON_PARAM_PINS);
   setParamName(FPSTR(STRING_PINS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);

   param = &_params[CYLON_PARAM_NUMPIXELS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, CYLON_PARAM_NUMPIXELS);
   setParamName(FPSTR(STRING_NUMPIXELS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   // @default numPixels=16
   _params[CYLON_PARAM_NUMPIXELS_E].data.uint8[0] = 16;
}


void CylonModule::disable() {
  Log.noticeln(F("[Cy.d]"));
  if (_strip) {
    _strip->SetBrightness(25);
    _strip->ClearTo(RgbColor(0,0,0));
  }
  DroneModule::disable();
  Log.noticeln(F("[Cy.d] end"));
}


void CylonModule::setup() {
  //Log.noticeln(F("[Cy.s]"));
  DroneModule::setup();  
  if (_strip == NULL) {
    _strip = _ds->requestStrip(_params[CYLON_PARAM_PINS_E].data.uint8[0], _params[CYLON_PARAM_NUMPIXELS_E].data.uint8[0], this);

    if (_strip != NULL) {
      _firstPixel = _ds->getStripFirstPixel(_params[CYLON_PARAM_PINS_E].data.uint8[0]);

      // init red while booting
      for(uint8_t i=_firstPixel; i<_params[CYLON_PARAM_NUMPIXELS_E].data.uint8[0]+_firstPixel; i++) {
        _strip->SetPixelColor(i, RgbColor(255, 0, 0));
      }
      _strip->Show();
    } else {
      Log.errorln(F("Unable to init strip on pin %d"), _params[CYLON_PARAM_PINS_E].data.uint8[0]);
      disable();
    }
  }
}


void CylonModule::loop() {
  DroneModule::loop();
  //Log.noticeln(F("[Cy.l]"));

  unsigned long loopTime = millis();

  if (_strip) {

    _strip->SetBrightness(255);

    uint8_t perSide = _params[CYLON_PARAM_NUMPIXELS_E].data.uint8[0] / 2;


    // update left strip (LEDs 0..7 + 1)
    uint8_t numLit = max(1.0f,round(perSide * fabs(_subs[CYLON_SUB_LEFT_E].param.data.f[0])));
    HsbColor blk = HsbColor(0,0,0);
    HsbColor c = HsbColor(0,0,0.5);

    for (uint8_t i=0; i<perSide; i++) {
      if (_subs[CYLON_SUB_LEFT_E].param.data.f[0] < 0) {
        // reverse = red
        c = HsbColor(0,1,0.5);
      } else {
        // fwd rainbow
        if (i == 0) {
          c = HsbColor(0,0,0.5);
        } else {
          c = HsbColor(0 + 0.7*i/perSide,1,0.5);
        }
      }
      _strip->SetPixelColor(i + _firstPixel, i<numLit ? c : blk);
    }


    // update right strip (LEDs 15..8 + 1)
    c = HsbColor(0,0,0.5);
    if (_subs[CYLON_SUB_RIGHT_E].param.data.f[0] < 0) c = HsbColor(0,1,0.5);
    numLit = max(1.0f,round(perSide * fabs(_subs[CYLON_SUB_RIGHT_E].param.data.f[0])));
    for (uint8_t i=0; i<perSide; i++) {
      if (_subs[CYLON_SUB_RIGHT_E].param.data.f[0] < 0) {
        // reverse = red
        c = HsbColor(0,1,0.5);
      } else {
        // fwd rainbow
        if (i == 0) {
          c = HsbColor(0,0,0.5);
        } else {
          c = HsbColor(0 + 0.7*i/perSide,1,0.5);
        }
      }

      _strip->SetPixelColor((_firstPixel-1) + _params[CYLON_PARAM_NUMPIXELS_E].data.uint8[0]-i, i<numLit ? c : blk);
    }
  }
}


void CylonModule::update() {
  if (!_setupDone) return;
}
