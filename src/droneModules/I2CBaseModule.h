/*

@type I2CBase
@inherits Drone
@description Base class for I2C devices

*/
#ifndef I2CBASE_MODULE_H
#define I2CBASE_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"

// @pub 8;u8;1;w;bus;I2C Bus 0..7
#define I2CBASE_PARAM_BUS      8

// @pub 9;u32;1;w;addr;I2C address
#define I2CBASE_PARAM_ADDR     9

#define I2CBASE_PARAM_BUS_E      0
#define I2CBASE_PARAM_ADDR_E     1

#define I2CBASE_PARAM_ENTRIES  2
#define I2CBASE_SUBCLASS_PARAM_START  10

// class
class I2CBaseModule:  public DroneModule {
protected:

public:

  I2CBaseModule(uint8_t id, DroneSystem* ds);

  void initBaseParams();

  //virtual void onParamWrite(DRONE_PARAM_ENTRY *param);

  virtual void doReset();
  virtual boolean isAlive();

  virtual void setup();

};

#endif
