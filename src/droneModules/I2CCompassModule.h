/*

@type          I2CCompass
@inherits      I2CBase
@description   Manages a generic I2C Compass, to be implemented in a concrete class e.g. I2CCompass

@config >>>
[I2CCompass = 6]
  name="Compass"
  interval= 50
  calibX= -6,0,6
  calibY= -6,0,6
  calibZ=-6,0,6
  trim= 180
  location= -1.8, 52, 100
  mode=1
  centre = -3.188, 4.167, -0.1043
  $location = @>GPS.location
  $roll = @>MPU6050.roll
  $pitch = @>MPU6050.pitch
  publish =heading, vector, calibX, calibY, calibZ
  publish = trim, mode, roll, pitch, raw
<<<

*/
#ifndef I2CCOMPASS_MODULE_H
#define I2CCOMPASS_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "I2Cdev.h"

#define I2CCOMPASS_I2C_ADDRESS  0x0D

// pubs
// pubs of form: <param address>;<type>;<number of values>;<name>;<description>

// @pub 10;f;4;r;vector;Magnetic field vector after pitch/roll compensation
#define I2CCOMPASS_PARAM_VECTOR          (I2CBASE_SUBCLASS_PARAM_START+0)
// @pub 11;f;1;r;heading;Heading adjusted for magnetic declination
#define I2CCOMPASS_PARAM_HEADING         (I2CBASE_SUBCLASS_PARAM_START+1)  //11
// @pub 12;f;1;r;declination;Current declination value
#define I2CCOMPASS_PARAM_DECLINATION     (I2CBASE_SUBCLASS_PARAM_START+2)  //12
// @pub 13;f;3;w;calibX;Min, center and max magnetic readings for the X axis
#define I2CCOMPASS_PARAM_CALIB_X         (I2CBASE_SUBCLASS_PARAM_START+3)  //13
// @pub 14;f;3;w;calibY;Min, center and max magnetic readings for the Y axis
#define I2CCOMPASS_PARAM_CALIB_Y         (I2CBASE_SUBCLASS_PARAM_START+4)  //14
// @pub 15;f;3;w;calibZ;Min, center and max magnetic readings for the Z axis
#define I2CCOMPASS_PARAM_CALIB_Z         (I2CBASE_SUBCLASS_PARAM_START+5)  //15

// @pub 18;f;1;w;trim;Manual calibration value to adjust heading to match hull (e.g. for a misaligned physical mount)
#define I2CCOMPASS_PARAM_TRIM            (I2CBASE_SUBCLASS_PARAM_START+8)  //18

// @ui enum; mode; online calibration, fixed calibration, reset calibration, store calibration
// @pub 20;u8;1;w;mode;Mode: 0=online calibration, 1=fixed calibration, 2=reset calibration, 3=store calibration
#define I2CCOMPASS_PARAM_MODE            (I2CBASE_SUBCLASS_PARAM_START+10)  // 20
// @pub 21;f;4;r;raw;Raw magnetic field vector
#define I2CCOMPASS_PARAM_RAW             (I2CBASE_SUBCLASS_PARAM_START+11)  // 21


#define I2CCOMPASS_PARAM_VECTOR_E          (I2CBASE_PARAM_ENTRIES+0)
#define I2CCOMPASS_PARAM_HEADING_E         (I2CBASE_PARAM_ENTRIES+1)
#define I2CCOMPASS_PARAM_DECLINATION_E     (I2CBASE_PARAM_ENTRIES+2)
#define I2CCOMPASS_PARAM_CALIB_X_E         (I2CBASE_PARAM_ENTRIES+3)
#define I2CCOMPASS_PARAM_CALIB_Y_E         (I2CBASE_PARAM_ENTRIES+4)
#define I2CCOMPASS_PARAM_CALIB_Z_E         (I2CBASE_PARAM_ENTRIES+5)
#define I2CCOMPASS_PARAM_TRIM_E            (I2CBASE_PARAM_ENTRIES+6)
#define I2CCOMPASS_PARAM_MODE_E            (I2CBASE_PARAM_ENTRIES+7)
#define I2CCOMPASS_PARAM_RAW_E             (I2CBASE_PARAM_ENTRIES+8)

#define I2CCOMPASS_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 9)

// subs
// subs of form: <param address>;<addr param address>;<type>;<number of values>;<name>;description

// @sub 16;17;f;3;location;Current location from GPS
#define I2CCOMPASS_SUB_LOCATION            (I2CBASE_SUBCLASS_PARAM_START+6) //16
#define I2CCOMPASS_SUB_LOCATION_ADDR       (I2CBASE_SUBCLASS_PARAM_START+7) //17
#define I2CCOMPASS_SUB_LOCATION_E          0

// @sub 22;23;f;1;pitch;Pitch from IMU (in degrees)
#define I2CCOMPASS_SUB_PITCH               (I2CBASE_SUBCLASS_PARAM_START+12) //22
#define I2CCOMPASS_SUB_PITCH_ADDR          (I2CBASE_SUBCLASS_PARAM_START+13) //23
#define I2CCOMPASS_SUB_PITCH_E             1

// @sub 24;25;f;1;roll;Roll from IMU (in degrees)
#define I2CCOMPASS_SUB_ROLL                (I2CBASE_SUBCLASS_PARAM_START+14) //24
#define I2CCOMPASS_SUB_ROLL_ADDR           (I2CBASE_SUBCLASS_PARAM_START+15) //25
#define I2CCOMPASS_SUB_ROLL_E              2

#define I2CCOMPASS_SUBS                    3

// strings
static const char I2CCOMPASS_STR_I2CCOMPASS[] PROGMEM = "I2CCompass";

#define I2CCOMPASS_MODE_ONLINE_CALIBRATION     0
#define I2CCOMPASS_MODE_FIXED_CALIBRATION      1
#define I2CCOMPASS_MODE_RESET_CALIBRATION      2
#define I2CCOMPASS_MODE_STORE_CALIBRATION      3


#define I2CCOMPASS_MOVING_AVERAGE_POINTS       10


// class
class I2CCompassModule:  public I2CBaseModule {
protected:
  int _location[2];  // lng, lat - rounded to whole digits

  // moving average on raw vector values
  float _raw[3];  // raw values straight from the sensor
  float _rawAvg[3];
  uint8_t _numRawSamples;

  // min and max limits for raw magnetic values in all orientations
  float _minRaw[3];
  float _maxRaw[3];
public:

  I2CCompassModule(uint8_t id, DroneSystem* ds);

  virtual boolean initSensor();

  virtual void getSensorValues();

  void setup();
  void update();

  void updateCalibrationValuesFromRaw();

  void loop();

};

#endif
