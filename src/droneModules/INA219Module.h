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

#define INA219_PARAM_SHUNTV          (I2CBASE_SUBCLASS_PARAM_START+0)
#define INA219_PARAM_BUSV            (I2CBASE_SUBCLASS_PARAM_START+1)
#define INA219_PARAM_CURRENT         (I2CBASE_SUBCLASS_PARAM_START+2)
#define INA219_PARAM_POWER           (I2CBASE_SUBCLASS_PARAM_START+3)
#define INA219_PARAM_LOADV           (I2CBASE_SUBCLASS_PARAM_START+4)
#define INA219_PARAM_CELLV           (I2CBASE_SUBCLASS_PARAM_START+5)
#define INA219_PARAM_ALARM           (I2CBASE_SUBCLASS_PARAM_START+6)
#define INA219_PARAM_CELLS           (I2CBASE_SUBCLASS_PARAM_START+7)
#define INA219_PARAM_THRESHOLD       (I2CBASE_SUBCLASS_PARAM_START+8)

#define INA219_PARAM_SHUNTV_E          (I2CBASE_PARAM_ENTRIES+0)
#define INA219_PARAM_BUSV_E            (I2CBASE_PARAM_ENTRIES+1)
#define INA219_PARAM_CURRENT_E         (I2CBASE_PARAM_ENTRIES+2)
#define INA219_PARAM_POWER_E           (I2CBASE_PARAM_ENTRIES+3)
#define INA219_PARAM_LOADV_E           (I2CBASE_PARAM_ENTRIES+4)
#define INA219_PARAM_CELLV_E           (I2CBASE_PARAM_ENTRIES+5)
#define INA219_PARAM_ALARM_E           (I2CBASE_PARAM_ENTRIES+6)
#define INA219_PARAM_CELLS_E           (I2CBASE_PARAM_ENTRIES+7)
#define INA219_PARAM_THRESHOLD_E       (I2CBASE_PARAM_ENTRIES+8)

#define INA219_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 9)


// strings
static const char INA219_STR_INA219[] PROGMEM = "INA219";


// class
class INA219Module:  public I2CBaseModule {
protected:
  Adafruit_INA219 *_sensor;
public:

  INA219Module(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);
  ~INA219Module();

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void doReset();

  void loadConfiguration(JsonObject &obj);

  void publishEntry(uint8_t i);

  void setup();
  void loop();


};

#endif
