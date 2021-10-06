#include "I2CBaseModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"


I2CBaseModule::I2CBaseModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   _bus = 0;
   _addr = 0;

   _loopInterval = 1000;
}


void I2CBaseModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

  _bus = obj[DRONE_STR_BUS] | _bus;
  _addr = obj[DRONE_STR_ADDR] | _addr;
}


void I2CBaseModule::doReset() {
  Log.warningln(F("I2C doReset"));
  if (_resetCount > 1) {
    // attempt resetting the multiplexer
    DroneWire::reset();
  }
  _resetCount++;
}


boolean I2CBaseModule::isAlive() {
  // if already in error state, then assume dead
  if (_error > 0) return false;

  // poll sensor to see if we're alive
  DroneWire::selectChannel(_bus);

  Wire.beginTransmission(_addr);
  byte err = Wire.endTransmission();
  if (err != 0) {
    setError(1);
  }
  return (err == 0);
}


void I2CBaseModule::setup() {
  DroneModule::setup();

  doReset();
}
