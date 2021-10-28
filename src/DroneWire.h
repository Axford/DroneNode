/*
   Utilities for handling the multiplexer
*/


#ifndef DRONE_WIRE_H
#define DRONE_WIRE_H

#include "Arduino.h"
#include <Wire.h>
#include <ArduinoLog.h>

#include "pinConfig.h"

#define TCAADDR_V1 0x70  // v1 motherboard
#define TCAADDR_V2 0x77  // v2+ motherboard

namespace DroneWire {

  void setup();

  void reset();

  void selectChannel(uint8_t chan);

  // scan current channel for devices
  void scan();

  void scanAll();
}

#endif
