/*

Manages a INA3221 I2C power monitor

*/
#ifndef INA3221_MODULE_H
#define INA3221_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "Beastdevices_INA3221.h"

#define INA3221_I2C_ADDRESS  0x40

#define INA3221_PARAM_CURRENT         (I2CBASE_SUBCLASS_PARAM_START+2)
#define INA3221_PARAM_POWER           (I2CBASE_SUBCLASS_PARAM_START+3)
#define INA3221_PARAM_LOADV           (I2CBASE_SUBCLASS_PARAM_START+4)
#define INA3221_PARAM_CELLV           (I2CBASE_SUBCLASS_PARAM_START+5)
#define INA3221_PARAM_ALARM           (I2CBASE_SUBCLASS_PARAM_START+6)
#define INA3221_PARAM_CELLS           (I2CBASE_SUBCLASS_PARAM_START+7)
#define INA3221_PARAM_THRESHOLD       (I2CBASE_SUBCLASS_PARAM_START+8)

#define INA3221_PARAM_CURRENT_E         (I2CBASE_PARAM_ENTRIES+0)
#define INA3221_PARAM_POWER_E           (I2CBASE_PARAM_ENTRIES+1)
#define INA3221_PARAM_LOADV_E           (I2CBASE_PARAM_ENTRIES+2)
#define INA3221_PARAM_CELLV_E           (I2CBASE_PARAM_ENTRIES+3)
#define INA3221_PARAM_ALARM_E           (I2CBASE_PARAM_ENTRIES+4)
#define INA3221_PARAM_CELLS_E           (I2CBASE_PARAM_ENTRIES+5)
#define INA3221_PARAM_THRESHOLD_E       (I2CBASE_PARAM_ENTRIES+6)

#define INA3221_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 7)


// strings
static const char INA3221_STR_INA3221[] PROGMEM = "INA3221";


// class
class INA3221Module:  public I2CBaseModule {
protected:
  Beastdevices_INA3221 *_sensor;
public:

  INA3221Module(uint8_t id, DroneSystem* ds);
  ~INA3221Module();

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void doReset();

  void publishEntry(uint8_t i);

  void setup();
  void loop();

  uint8_t diagnosticDisplays();
  void drawDiagnosticDisplay(SSD1306Wire *display, uint8_t page);

};

#endif
