#include "DroneLED.h"
#include "DroneSystem.h"

DroneLED::DroneLED(DroneSystem* ds) {
  _ds = ds;
  _state = DRONE_LED_STATE_STARTUP;
  _strip = NULL;
  _animationTimer = 0;
  _animationState = 0;

  if (ds->motherboardVersion() >= 4) {
    _hw = DRONE_LED_HW_NEOPIXEL;
  } else {
    _hw = DRONE_LED_HW_BUILTIN;
  }

  // init
  if (_hw == DRONE_LED_HW_BUILTIN) {
    // init built-in LED
    pinMode(DRONE_LED_PIN, OUTPUT);
  } else {
    // init NeoPixel strip
    _strip = new NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>(DRONE_LED_MAX_PIXELS, DRONE_LED_PIN);

    _strip->Begin();
    _strip->Show();
    // default brightness is very low to save battery
    _strip->SetBrightness(15);
  }

  // set initial state
  setState(DRONE_LED_STATE_STARTUP);
}


boolean DroneLED::isNeopixel() {
  return (_hw == DRONE_LED_HW_NEOPIXEL);
}


void DroneLED::setState(uint8_t newState) {
  _state = newState;
  switch (newState) {
    case DRONE_LED_STATE_STARTUP:
      if (_hw == DRONE_LED_HW_NEOPIXEL) {
        _strip->SetPixelColor(0, RgbColor(255, 255, 255));
      } else {
        digitalWrite(DRONE_LED_PIN, HIGH);
      }
      break;

    case DRONE_LED_STATE_ERROR:
      if (_hw == DRONE_LED_HW_NEOPIXEL) {
        _strip->SetPixelColor(0, RgbColor(255, 0, 0));
      } else {
        digitalWrite(DRONE_LED_PIN, HIGH);
      }
      break;


    case DRONE_LED_STATE_RUNNING_NO_WIFI:
      if (_hw == DRONE_LED_HW_NEOPIXEL) {
        _strip->SetPixelColor(0, RgbColor(0, 0, 255));
      } else {
        digitalWrite(DRONE_LED_PIN, HIGH);
      }
      break;

    case DRONE_LED_STATE_RUNNING_WIFI:
      if (_hw == DRONE_LED_HW_NEOPIXEL) {
        _strip->SetPixelColor(0, RgbColor(0, 50, 0));
      } else {
        digitalWrite(DRONE_LED_PIN, LOW);
      }
      break;

    case DRONE_LED_STATE_UPDATING:
      if (_hw == DRONE_LED_HW_NEOPIXEL) {
        _strip->SetPixelColor(0, RgbColor(0, 255, 255));
      } else {
        digitalWrite(DRONE_LED_PIN, LOW);
      }
      break;

    case DRONE_LED_STATE_RESTART:
      if (_hw == DRONE_LED_HW_NEOPIXEL) {
        _strip->SetPixelColor(0, RgbColor(20, 0, 0));
      } else {
        digitalWrite(DRONE_LED_PIN, LOW);
      }
      break;
  }

  update();
}


void DroneLED::update() {
  if (_hw == DRONE_LED_HW_NEOPIXEL) {

    uint32_t loopTime = millis();

    if (_state == DRONE_LED_STATE_UPDATING) {
      // animate to show update in progress
      if (loopTime > _animationTimer + 200) {
        if (_animationState == 0) {
          _animationState = 1;
          _strip->SetPixelColor(0, RgbColor(120, 50, 255));
        } else {
          _animationState = 0;
          _strip->SetPixelColor(0, RgbColor(50, 0, 100));
        }
        
        _animationTimer = loopTime;
      }
    }
    _strip->Show();
  }
}


void DroneLED::loop() {
  update();
}
