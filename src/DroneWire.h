/*
   Utilities for handling the multiplexer
*/


#ifndef DRONE_WIRE_H
#define DRONE_WIRE_H

#include "Arduino.h"
#include <Wire.h>
#include <ArduinoLog.h>
#include <ESPAsyncWebServer.h>

#include "pinConfig.h"

#define TCAADDR_V1 0x70  // v1 motherboard
#define TCAADDR_V2 0x77  // v2+ motherboard

namespace DroneWire {

  void setup();

  uint8_t getMultiplexerVersion();

  void reset();

  void selectChannel(uint8_t chan);

  boolean scanAddress(uint8_t addr);

  // scan current channel for devices
  void scan();

  void scanAll();

  void serveScanInfo(AsyncResponseStream *response);
  void serveScanAllInfo(AsyncWebServerRequest *request);
}

#endif
