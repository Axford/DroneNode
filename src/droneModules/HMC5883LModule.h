/*

Manages a HMC5883L I2C Compass

*/
#ifndef HMC5883L_MODULE_H
#define HMC5883L_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

//#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883_U.h>

#include "I2Cdev.h"
#include "HMC5883L.h"

#define HMC5883L_I2C_ADDRESS  0x1E  // write address, read address is +1

// pubs
#define HMC5883L_PARAM_VECTOR          (I2CBASE_SUBCLASS_PARAM_START+0)  //10
#define HMC5883L_PARAM_HEADING         (I2CBASE_SUBCLASS_PARAM_START+1)  // 11
#define HMC5883L_PARAM_DECLINATION     (I2CBASE_SUBCLASS_PARAM_START+2)
#define HMC5883L_PARAM_CALIB_X         (I2CBASE_SUBCLASS_PARAM_START+3)
#define HMC5883L_PARAM_CALIB_Y         (I2CBASE_SUBCLASS_PARAM_START+4)
#define HMC5883L_PARAM_TRIM            (I2CBASE_SUBCLASS_PARAM_START+5)

#define HMC5883L_PARAM_VECTOR_E          (I2CBASE_PARAM_ENTRIES+0)
#define HMC5883L_PARAM_HEADING_E         (I2CBASE_PARAM_ENTRIES+1)
#define HMC5883L_PARAM_DECLINATION_E     (I2CBASE_PARAM_ENTRIES+2)
#define HMC5883L_PARAM_CALIB_X_E         (I2CBASE_PARAM_ENTRIES+3)
#define HMC5883L_PARAM_CALIB_Y_E         (I2CBASE_PARAM_ENTRIES+4)
#define HMC5883L_PARAM_TRIM_E            (I2CBASE_PARAM_ENTRIES+5)

#define HMC5883L_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 6)

// subs
#define HMC5883L_SUB_LOCATION            (I2CBASE_SUBCLASS_PARAM_START+5)
#define HMC5883L_SUB_LOCATION_ADDR       (I2CBASE_SUBCLASS_PARAM_START+6)
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

  HMC5883LModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);
  ~HMC5883LModule();

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void doReset();

  //void publishEntry(uint8_t i);

  void setup();
  void update();
  void loop();


};

#endif
