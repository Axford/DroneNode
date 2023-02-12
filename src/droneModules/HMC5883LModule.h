/*

@type          HMC5883L
@inherits      I2CBase
@description   Manages a HMC5883L I2C Compass

@config >>>
[HMC5883L= 8]
  name= "Compass"
  interval= 100
  bus= 3
  status= 1
  calibX= -0.06, 2.88, 5.82
  calibY= -6.4, -3.48, -0.56
  trim= 0
  location= -1.8, 52, 100
  $location= @>7.8
  publish = heading, vector, calibX, calibY
<<<

*/
#ifndef HMC5883L_MODULE_H
#define HMC5883L_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "I2Cdev.h"
#include "HMC5883L.h"

#define HMC5883L_I2C_ADDRESS  0x1E  // write address, read address is +1

// pubs
// pubs of form: <param address>;<type>;<number of values>;<name>;<description>

// @pub 10;f;4;vector;Raw magnetic field vector
#define HMC5883L_PARAM_VECTOR          (I2CBASE_SUBCLASS_PARAM_START+0)
// @pub 11;f;1;heading;Heading adjusted for magnetic declination
#define HMC5883L_PARAM_HEADING         (I2CBASE_SUBCLASS_PARAM_START+1)  //11
// @pub 12;f;1;declination;Current declination value
#define HMC5883L_PARAM_DECLINATION     (I2CBASE_SUBCLASS_PARAM_START+2)  //12
// @pub 13;f;3;calibX;Min, center and max magnetic readings for the X axis
#define HMC5883L_PARAM_CALIB_X         (I2CBASE_SUBCLASS_PARAM_START+3)  //13
// @pub 14;f;3;calibY;Min, center and max magnetic readings for the Y axis
#define HMC5883L_PARAM_CALIB_Y         (I2CBASE_SUBCLASS_PARAM_START+4)  //14
// @pub 15;f;1;trim;Manual calibration value to adjust heading to match hull (e.g. for a misaligned physical mount)
#define HMC5883L_PARAM_TRIM            (I2CBASE_SUBCLASS_PARAM_START+5)  //15
// @pub 18;f;4;limits;Averaged limits at the four quadrants, used to refine the calibration onoine
#define HMC5883L_PARAM_LIMITS          (I2CBASE_SUBCLASS_PARAM_START+8)  // 18
// @pub 19;u32;4;samples;Number of calibration samples per quadrant
#define HMC5883L_PARAM_SAMPLES         (I2CBASE_SUBCLASS_PARAM_START+9)  // 19

#define HMC5883L_PARAM_VECTOR_E          (I2CBASE_PARAM_ENTRIES+0)
#define HMC5883L_PARAM_HEADING_E         (I2CBASE_PARAM_ENTRIES+1)
#define HMC5883L_PARAM_DECLINATION_E     (I2CBASE_PARAM_ENTRIES+2)
#define HMC5883L_PARAM_CALIB_X_E         (I2CBASE_PARAM_ENTRIES+3)
#define HMC5883L_PARAM_CALIB_Y_E         (I2CBASE_PARAM_ENTRIES+4)
#define HMC5883L_PARAM_TRIM_E            (I2CBASE_PARAM_ENTRIES+5)
#define HMC5883L_PARAM_LIMITS_E          (I2CBASE_PARAM_ENTRIES+6)
#define HMC5883L_PARAM_SAMPLES_E         (I2CBASE_PARAM_ENTRIES+7)

#define HMC5883L_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 8)

// subs
// subs of form: <param address>;<addr param address>;<type>;<number of values>;<name>;description

// @sub 16;17;f;2;location;Current location from GPS
#define HMC5883L_SUB_LOCATION            (I2CBASE_SUBCLASS_PARAM_START+6) //16
#define HMC5883L_SUB_LOCATION_ADDR       (I2CBASE_SUBCLASS_PARAM_START+7) //17
#define HMC5883L_SUB_LOCATION_E          0

#define HMC5883L_SUBS                    1

// strings
static const char HMC5883L_STR_HMC5883L[] PROGMEM = "HMC5883L";


// class
class HMC5883LModule:  public I2CBaseModule {
protected:
  int _location[2];  // lng, lat - rounded to whole digits
  //DRONE_LINK_ADDR _locationInput;
  HMC5883L *_sensor;
public:

  HMC5883LModule(uint8_t id, DroneSystem* ds);
  ~HMC5883LModule();

  void doReset();

  //void publishEntry(uint8_t i);

  void setup();
  void update();
  void loop();

  void updateQuadrant(uint8_t quadrant, float v);


};

#endif
