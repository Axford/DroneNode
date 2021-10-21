/*

Manages a I2CBASE I2C power monitor

*/
#ifndef I2CBASE_MODULE_H
#define I2CBASE_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"

#define I2CBASE_PARAM_BUS      8
#define I2CBASE_PARAM_ADDR     9

#define I2CBASE_PARAM_BUS_E      0
#define I2CBASE_PARAM_ADDR_E     1

#define I2CBASE_PARAM_ENTRIES  2
#define I2CBASE_SUBCLASS_PARAM_START  10

// class
class I2CBaseModule:  public DroneModule {
protected:

public:

  I2CBaseModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  void initBaseParams();

  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  //virtual void onParamWrite(DRONE_PARAM_ENTRY *param);

  virtual void doReset();
  virtual boolean isAlive();

  virtual void setup();
};

#endif
