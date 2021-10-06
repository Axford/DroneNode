/*

Manages a INA219 I2C power monitor

*/
#ifndef INA219_MODULE_H
#define INA219_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>

#define INA219_I2C_ADDRESS  0x40

#define INA219_PARAM_SHUNTV          B00001000
#define INA219_PARAM_BUSV            B00010000
#define INA219_PARAM_CURRENT         B00100000
#define INA219_PARAM_POWER           B01000000
#define INA219_PARAM_LOADV           B10000000

#define INA219_PARAM_SHUNTV_E          0
#define INA219_PARAM_BUSV_E            1
#define INA219_PARAM_CURRENT_E         2
#define INA219_PARAM_POWER_E           3
#define INA219_PARAM_LOADV_E           4

#define INA219_PARAM_ENTRIES           5

// strings
static const char INA219_STR_INA219[] PROGMEM = "INA219";


// class
class INA219Module:  public I2CBaseModule {
protected:
  Adafruit_INA219 *_sensor;
public:

  INA219Module(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm);
  ~INA219Module();

  void doReset();

  void loadConfiguration(JsonObject &obj);

  void publishEntry(uint8_t i);

  virtual void loop();


};

#endif
