/*

@type          LSM9DS1
@inherits      I2CBaseModule
@category      Input.Compass
@description   Manages a LSM9DS1 I2C Compass

@config >>>
[LSM9DS1 = 6]
  name= "Compass"
  interval= 200
  bus= 0
  status= 1
  calibX= -3.5, 0, 2.3
  calibY= -3.6, 0, 1.7
  location= -1.8, 52, 100
  $location = @>GPS.location
  publish = heading, vector, calibX, calibY
<<<

*/
#ifndef LSM9DS1_MODULE_H
#define LSM9DS1_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "I2Cdev.h"
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

/*
@I2CAddress        0x1E
@default addr = 30
@I2CAddress        0x6B
*/ 
#define LSM9DS1_I2C_ADDRESS_1  0x1E
#define LSM9DS1_I2C_ADDRESS_2  0x6B

// pubs
// pubs of form: <param address>;<type>;<number of values>;<name>;<description>

// @pub 10;f;4;r;vector;Raw magnetic field vector
#define LSM9DS1_PARAM_VECTOR          (I2CBASE_SUBCLASS_PARAM_START+0)
// @pub 11;f;1;r;heading;Heading adjusted for magnetic declination
#define LSM9DS1_PARAM_HEADING         (I2CBASE_SUBCLASS_PARAM_START+1)  //11
// @pub 12;f;1;w;declination;Current declination value
#define LSM9DS1_PARAM_DECLINATION     (I2CBASE_SUBCLASS_PARAM_START+2)  //12
// @pub 13;f;3;w;calibX;Min, center and max magnetic readings for the X axis
#define LSM9DS1_PARAM_CALIB_X         (I2CBASE_SUBCLASS_PARAM_START+3)  //13
// @pub 14;f;3;w;calibY;Min, center and max magnetic readings for the Y axis
#define LSM9DS1_PARAM_CALIB_Y         (I2CBASE_SUBCLASS_PARAM_START+4)  //14
// @pub 15;f;1;w;trim;Manual calibration value to adjust heading to match hull (e.g. for a misaligned physical mount)
#define LSM9DS1_PARAM_TRIM            (I2CBASE_SUBCLASS_PARAM_START+5)  //15
// @pub 18;f;4;w;limits;Averaged limits at the four quadrants, used to refine the calibration onoine
#define LSM9DS1_PARAM_LIMITS            (I2CBASE_SUBCLASS_PARAM_START+8)  // 18

#define LSM9DS1_PARAM_VECTOR_E          (I2CBASE_PARAM_ENTRIES+0)
#define LSM9DS1_PARAM_HEADING_E         (I2CBASE_PARAM_ENTRIES+1)
#define LSM9DS1_PARAM_DECLINATION_E     (I2CBASE_PARAM_ENTRIES+2)
#define LSM9DS1_PARAM_CALIB_X_E         (I2CBASE_PARAM_ENTRIES+3)
#define LSM9DS1_PARAM_CALIB_Y_E         (I2CBASE_PARAM_ENTRIES+4)
#define LSM9DS1_PARAM_TRIM_E            (I2CBASE_PARAM_ENTRIES+5)
#define LSM9DS1_PARAM_LIMITS_E          (I2CBASE_PARAM_ENTRIES+6)

#define LSM9DS1_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 7)

// subs
// subs of form: <param address>;<addr param address>;<type>;<number of values>;<name>;description

// @sub 16;17;f;2;location;Current location from GPS
#define LSM9DS1_SUB_LOCATION            (I2CBASE_SUBCLASS_PARAM_START+6) //16
#define LSM9DS1_SUB_LOCATION_ADDR       (I2CBASE_SUBCLASS_PARAM_START+7) //17
#define LSM9DS1_SUB_LOCATION_E          0

#define LSM9DS1_SUBS                    1

// strings
static const char LSM9DS1_STR_LSM9DS1[] PROGMEM = "LSM9DS1";


// class
class LSM9DS1Module:  public I2CBaseModule {
protected:
  int _location[2];  // lng, lat - rounded to whole digits
  //DRONE_LINK_ADDR _locationInput;
  Adafruit_LSM9DS1 *_sensor;
public:

  LSM9DS1Module(uint8_t id, DroneSystem* ds);
  ~LSM9DS1Module();

  void doReset();

  //void publishEntry(uint8_t i);

  void setup();
  void update();
  void loop();


};

#endif
