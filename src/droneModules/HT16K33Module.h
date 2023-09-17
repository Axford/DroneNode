/*

@type          HT16K33
@inherits      I2CBase
@description   Manages a HT16K33 7 segment display to show uptime

@config >>>
[HT16K33= 8]
  name= "7Seg"
  bus= 3
<<<

*/
#ifndef HT16K33_MODULE_H
#define HT16K33_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "I2Cdev.h"

#include "HT16K33.h"

/*
@I2CAddress        0x70
*/ 
#define HT16K33_I2C_ADDRESS  0x70  // default address


// pubs
// pubs of form: <param address>;<type>;<number of values>;<name>;<description>

#define HT16K33_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 0)

// subs
// subs of form: <param address>;<addr param address>;<type>;<number of values>;<name>;description

#define HT16K33_SUBS                    0

// strings
static const char HT16K33_STR_HT16K33[] PROGMEM = "HT16K33";


// class
class HT16K33Module:  public I2CBaseModule {
protected:
  HT16K33 * _sensor;
public:

  HT16K33Module(uint8_t id, DroneSystem* ds);

  void doReset();

  void setup();
  void loop();
};

#endif
