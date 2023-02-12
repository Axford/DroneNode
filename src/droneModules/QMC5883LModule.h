/*

@type          QMC5883L
@inherits      I2CBase
@description   Manages a QMC5883L I2C Compass

@config >>>
QMC5883L.new 6
  name "Compass"
  interval 200
  bus 0
  status 1
  calibX -3.5 0 2.3
  calibY -3.6 0 1.7
  // default location
  location -1.8 52 100
  //$location [@>GPS.location]
  $location [@>5.8]
  .publish "heading"
  .publish "vector"
  .publish "calibX"
  .publish "calibY"
  .publish "samples"
.done
<<<

*/
#ifndef QMC5883L_MODULE_H
#define QMC5883L_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "I2Cdev.h"
#include "QMC5883LCompass.h"

#define QMC5883L_I2C_ADDRESS  0x0D

// pubs
// pubs of form: <param address>;<type>;<number of values>;<name>;<description>

// @pub 10;f;4;vector;Raw magnetic field vector
#define QMC5883L_PARAM_VECTOR          (I2CBASE_SUBCLASS_PARAM_START+0)
// @pub 11;f;1;heading;Heading adjusted for magnetic declination
#define QMC5883L_PARAM_HEADING         (I2CBASE_SUBCLASS_PARAM_START+1)  //11
// @pub 12;f;1;declination;Current declination value
#define QMC5883L_PARAM_DECLINATION     (I2CBASE_SUBCLASS_PARAM_START+2)  //12
// @pub 13;f;3;calibX;Min, center and max magnetic readings for the X axis
#define QMC5883L_PARAM_CALIB_X         (I2CBASE_SUBCLASS_PARAM_START+3)  //13
// @pub 14;f;3;calibY;Min, center and max magnetic readings for the Y axis
#define QMC5883L_PARAM_CALIB_Y         (I2CBASE_SUBCLASS_PARAM_START+4)  //14
// @pub 15;f;1;trim;Manual calibration value to adjust heading to match hull (e.g. for a misaligned physical mount)
#define QMC5883L_PARAM_TRIM            (I2CBASE_SUBCLASS_PARAM_START+5)  //15
// @pub 18;f;4;limits;Averaged limits at the four quadrants, used to refine the calibration onoine
#define QMC5883L_PARAM_LIMITS            (I2CBASE_SUBCLASS_PARAM_START+8)  // 18
// @pub 19;u32;4;samples;Number of calibration samples per quadrant
#define QMC5883L_PARAM_SAMPLES         (I2CBASE_SUBCLASS_PARAM_START+9)  // 19

#define QMC5883L_PARAM_VECTOR_E          (I2CBASE_PARAM_ENTRIES+0)
#define QMC5883L_PARAM_HEADING_E         (I2CBASE_PARAM_ENTRIES+1)
#define QMC5883L_PARAM_DECLINATION_E     (I2CBASE_PARAM_ENTRIES+2)
#define QMC5883L_PARAM_CALIB_X_E         (I2CBASE_PARAM_ENTRIES+3)
#define QMC5883L_PARAM_CALIB_Y_E         (I2CBASE_PARAM_ENTRIES+4)
#define QMC5883L_PARAM_TRIM_E            (I2CBASE_PARAM_ENTRIES+5)
#define QMC5883L_PARAM_LIMITS_E          (I2CBASE_PARAM_ENTRIES+6)
#define QMC5883L_PARAM_SAMPLES_E         (I2CBASE_PARAM_ENTRIES+7)

#define QMC5883L_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 8)

// subs
// subs of form: <param address>;<addr param address>;<type>;<number of values>;<name>;description

// @sub 16;17;f;2;location;Current location from GPS
#define QMC5883L_SUB_LOCATION            (I2CBASE_SUBCLASS_PARAM_START+6) //16
#define QMC5883L_SUB_LOCATION_ADDR       (I2CBASE_SUBCLASS_PARAM_START+7) //17
#define QMC5883L_SUB_LOCATION_E          0

#define QMC5883L_SUBS                    1

// strings
static const char QMC5883L_STR_QMC5883L[] PROGMEM = "QMC5883L";


// class
class QMC5883LModule:  public I2CBaseModule {
protected:
  int _location[2];  // lng, lat - rounded to whole digits
  //DRONE_LINK_ADDR _locationInput;
  QMC5883LCompass *_sensor;
public:

  QMC5883LModule(uint8_t id, DroneSystem* ds);
  ~QMC5883LModule();

  void doReset();

  //void publishEntry(uint8_t i);

  void setup();
  void update();
  void loop();

  void updateQuadrant(uint8_t quadrant, float v);

};

#endif
