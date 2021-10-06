/*
   Utilities for handling the multiplexer
*/


#ifndef DRONE_WIRE_H
#define DRONE_WIRE_H

#include "Arduino.h"
#include <Wire.h>
#include <ArduinoLog.h>

#include "pinConfig.h"

#define TCAADDR 0x70

namespace DroneWire {

  void setup();

  void reset();

  void selectChannel(uint8_t chan);

  // scan current channel for devices
  void scan();

  void scanAll();
}

#endif
