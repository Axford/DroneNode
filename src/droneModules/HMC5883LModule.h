/*

Manages a HMC5883L I2C power monitor

*/
#ifndef HMC5883L_MODULE_H
#define HMC5883L_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#define HMC5883L_I2C_ADDRESS  0x1E  // write address, read address is +1

// pubs
#define HMC5883L_PARAM_VECTOR          B00001000
#define HMC5883L_PARAM_HEADING         B00010000
#define HMC5883L_PARAM_DECLINATION     B00100000
#define HMC5883L_PARAM_CALIB_X         B01000000
#define HMC5883L_PARAM_CALIB_Y         B10000000

#define HMC5883L_PARAM_VECTOR_E          0
#define HMC5883L_PARAM_HEADING_E         1
#define HMC5883L_PARAM_DECLINATION_E     2
#define HMC5883L_PARAM_CALIB_X_E         3
#define HMC5883L_PARAM_CALIB_Y_E         4

#define HMC5883L_PARAM_ENTRIES           5

// subs
#define HMC5883L_SUB_LOCATION            9
#define HMC5883L_SUB_LOCATION_ADDR       10
#define HMC5883L_SUB_LOCATION_E          0

#define HMC5883L_SUBS                    1

// strings
static const char HMC5883L_STR_HMC5883L[] PROGMEM = "HMC5883L";


// class
class HMC5883LModule:  public I2CBaseModule {
protected:
  int _location[2];  // lng, lat - rounded to whole digits
  //DRONE_LINK_ADDR _locationInput;
  Adafruit_HMC5883_Unified *_sensor;
public:

  HMC5883LModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm);
  ~HMC5883LModule();

  void doReset();

  void loadConfiguration(JsonObject &obj);

  void publishEntry(uint8_t i);

  void update();

  void loop();


};

#endif
