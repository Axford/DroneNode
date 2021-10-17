/*

Manages a I2CBASE I2C power monitor

*/
#ifndef I2CBASE_MODULE_H
#define I2CBASE_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"


// class
class I2CBaseModule:  public DroneModule {
protected:
  uint8_t _bus;  // I2C bus number 0..7
  uint8_t _addr;  // I2C address
public:

  I2CBaseModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);

  //virtual void onParamWrite(DRONE_PARAM_ENTRY *param);

  virtual void loadConfiguration(JsonObject &obj);

  virtual void doReset();
  virtual boolean isAlive();

  virtual void setup();
};

#endif
