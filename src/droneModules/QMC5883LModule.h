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

// @pub 10;f;4;vector;Magnetic field vector after pitch/roll compensation
#define QMC5883L_PARAM_VECTOR          (I2CBASE_SUBCLASS_PARAM_START+0)
// @pub 11;f;1;heading;Heading adjusted for magnetic declination
#define QMC5883L_PARAM_HEADING         (I2CBASE_SUBCLASS_PARAM_START+1)  //11
// @pub 12;f;1;declination;Current declination value
#define QMC5883L_PARAM_DECLINATION     (I2CBASE_SUBCLASS_PARAM_START+2)  //12
// @pub 13;f;3;calibX;Min, center and max magnetic readings for the X axis
#define QMC5883L_PARAM_CALIB_X         (I2CBASE_SUBCLASS_PARAM_START+3)  //13
// @pub 14;f;3;calibY;Min, center and max magnetic readings for the Y axis
#define QMC5883L_PARAM_CALIB_Y         (I2CBASE_SUBCLASS_PARAM_START+4)  //14
// @pub 15;f;3;calibZ;Min, center and max magnetic readings for the Z axis
#define QMC5883L_PARAM_CALIB_Z         (I2CBASE_SUBCLASS_PARAM_START+5)  //15

// @pub 18;f;1;trim;Manual calibration value to adjust heading to match hull (e.g. for a misaligned physical mount)
#define QMC5883L_PARAM_TRIM            (I2CBASE_SUBCLASS_PARAM_START+8)  //18
// @pub 20;u8;1;mode;Mode: 0=online calibration, 1=fixed calibration, 2=reset calibration, 3=store calibration
#define QMC5883L_PARAM_MODE            (I2CBASE_SUBCLASS_PARAM_START+10)  // 20
// @pub 21;f;4;raw;Raw magnetic field vector
#define QMC5883L_PARAM_RAW             (I2CBASE_SUBCLASS_PARAM_START+11)  // 21


#define QMC5883L_PARAM_VECTOR_E          (I2CBASE_PARAM_ENTRIES+0)
#define QMC5883L_PARAM_HEADING_E         (I2CBASE_PARAM_ENTRIES+1)
#define QMC5883L_PARAM_DECLINATION_E     (I2CBASE_PARAM_ENTRIES+2)
#define QMC5883L_PARAM_CALIB_X_E         (I2CBASE_PARAM_ENTRIES+3)
#define QMC5883L_PARAM_CALIB_Y_E         (I2CBASE_PARAM_ENTRIES+4)
#define QMC5883L_PARAM_CALIB_Z_E         (I2CBASE_PARAM_ENTRIES+5)
#define QMC5883L_PARAM_TRIM_E            (I2CBASE_PARAM_ENTRIES+6)
#define QMC5883L_PARAM_MODE_E            (I2CBASE_PARAM_ENTRIES+7)
#define QMC5883L_PARAM_RAW_E             (I2CBASE_PARAM_ENTRIES+8)

#define QMC5883L_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 9)

// subs
// subs of form: <param address>;<addr param address>;<type>;<number of values>;<name>;description

// @sub 16;17;f;2;location;Current location from GPS
#define QMC5883L_SUB_LOCATION            (I2CBASE_SUBCLASS_PARAM_START+6) //16
#define QMC5883L_SUB_LOCATION_ADDR       (I2CBASE_SUBCLASS_PARAM_START+7) //17
#define QMC5883L_SUB_LOCATION_E          0

// @sub 22;23;f;1;pitch;Pitch from IMU (in degrees)
#define QMC5883L_SUB_PITCH               (I2CBASE_SUBCLASS_PARAM_START+12) //22
#define QMC5883L_SUB_PITCH_ADDR          (I2CBASE_SUBCLASS_PARAM_START+13) //23
#define QMC5883L_SUB_PITCH_E             1

// @sub 24;25;f;1;roll;Roll from IMU (in degrees)
#define QMC5883L_SUB_ROLL                (I2CBASE_SUBCLASS_PARAM_START+14) //24
#define QMC5883L_SUB_ROLL_ADDR           (I2CBASE_SUBCLASS_PARAM_START+15) //25
#define QMC5883L_SUB_ROLL_E              2

#define QMC5883L_SUBS                    3

// strings
static const char QMC5883L_STR_QMC5883L[] PROGMEM = "QMC5883L";

#define QMC5883L_MODE_ONLINE_CALIBRATION     0
#define QMC5883L_MODE_FIXED_CALIBRATION      1
#define QMC5883L_MODE_RESET_CALIBRATION      2
#define QMC5883L_MODE_STORE_CALIBRATION      3


#define QMC5883L_MOVING_AVERAGE_POINTS       10


// class
class QMC5883LModule:  public I2CBaseModule {
protected:
  int _location[2];  // lng, lat - rounded to whole digits
  //DRONE_LINK_ADDR _locationInput;
  QMC5883LCompass *_sensor;

  // moving average on raw vector values
  float _rawAvg[3];
  uint8_t _numRawSamples;

  // min and max limits for raw magnetic values in all orientations
  float _minRaw[3];
  float _maxRaw[3];
public:

  QMC5883LModule(uint8_t id, DroneSystem* ds);
  ~QMC5883LModule();

  void doReset();

  //void publishEntry(uint8_t i);

  void setup();
  void update();

  void updateCalibrationValuesFromRaw();

  void loop();

};

#endif
