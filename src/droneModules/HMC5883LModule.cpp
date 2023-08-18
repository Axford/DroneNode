#include "HMC5883LModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"


HMC5883LModule::HMC5883LModule(uint8_t id, DroneSystem* ds):
  I2CCompassModule ( id, ds )
 {
   setTypeName(FPSTR(HMC5883L_STR_HMC5883L));
   //_params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = HMC5883L_I2C_ADDRESS;
   _sensor = NULL;
   
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = HMC5883L_I2C_ADDRESS;
}


boolean HMC5883LModule::initSensor() {
  if (!_sensor) {
    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
    
    // test to see if sensor responds on correct address:
    if (!DroneWire::scanAddress(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0])) {
      Log.errorln("[HMC5883L] Module not detected on I2C bus");
      setError(1);
      disable();
      return false;
    }

    _sensor = new HMC5883L(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]);
    _sensor->initialize();

    if (!_sensor->testConnection()) {
      Log.errorln("[HMC5883L] Failed comms test");
      setError(1);
      disable();
      return false;
    }
  }
  
  return true;
}


void HMC5883LModule::getSensorValues() {
  if (!_sensor) return;
  
  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  int16_t mx, my, mz;
  _sensor->getHeading(&mx, &my, &mz);

  _raw[0] = mx / 100.0;
  _raw[1] = my / 100.0;
  _raw[2] = mz / 100.0;
}


