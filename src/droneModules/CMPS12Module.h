/*

@type          CMPS12
@inherits      I2CBase
@description   Manages a CMPS12 I2C Compass

Datasheet: https://static.rapidonline.com/pdf/70-6398_v1.pdf

@config >>>
[CMPS12= 8]
  name= "Compass"
  interval= 100
  bus= 3
  status= 1
  trim= 0
  location= -1.8, 52, 100
  $location= @>7.8
  publish = heading, trim
<<<

*/
#ifndef CMPS12_MODULE_H
#define CMPS12_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "I2Cdev.h"

/*
@I2CAddress        0x60
*/ 
#define CMPS12_I2C_ADDRESS  0x60  // default address

#define CMPS12_ANGLE_8      1 // 8bit Angle register

// pubs
// pubs of form: <param address>;<type>;<number of values>;<name>;<description>


// @pub 10;f;1;r;heading;Heading adjusted for magnetic declination
#define CMPS12_PARAM_HEADING         (I2CBASE_SUBCLASS_PARAM_START+0)  //10
// @pub 11;f;1;r;declination;Current declination value
#define CMPS12_PARAM_DECLINATION     (I2CBASE_SUBCLASS_PARAM_START+1)  //11
// @pub 12;f;1;w;trim;Manual calibration value to adjust heading to match hull (e.g. for a misaligned physical mount)
#define CMPS12_PARAM_TRIM            (I2CBASE_SUBCLASS_PARAM_START+2)  //12
// @pub 13;f;2;r;vector;Pitch and roll vector
#define CMPS12_PARAM_VECTOR          (I2CBASE_SUBCLASS_PARAM_START+3)  //13


#define CMPS12_PARAM_HEADING_E         (I2CBASE_PARAM_ENTRIES+0)
#define CMPS12_PARAM_DECLINATION_E     (I2CBASE_PARAM_ENTRIES+1)
#define CMPS12_PARAM_TRIM_E            (I2CBASE_PARAM_ENTRIES+2)
#define CMPS12_PARAM_VECTOR_E          (I2CBASE_PARAM_ENTRIES+3)

#define CMPS12_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 4)

// subs
// subs of form: <param address>;<addr param address>;<type>;<number of values>;<name>;description

// @sub 20;21;f;2;location;Current location from GPS
#define CMPS12_SUB_LOCATION            (I2CBASE_SUBCLASS_PARAM_START+10) //20
#define CMPS12_SUB_LOCATION_ADDR       (I2CBASE_SUBCLASS_PARAM_START+11) //21
#define CMPS12_SUB_LOCATION_E          0

#define CMPS12_SUBS                    1

// strings
static const char CMPS12_STR_CMPS12[] PROGMEM = "CMPS12";


// class
class CMPS12Module:  public I2CBaseModule {
protected:
  int _location[2];  // lng, lat - rounded to whole digits

public:

  CMPS12Module(uint8_t id, DroneSystem* ds);

  void doReset();

  void setup();
  void update();
  void loop();
};

#endif
