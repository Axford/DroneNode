#include "QMC5883LModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include <SPIFFS.h>
#include "../navMath.h"
#include "Preferences.h"


QMC5883LModule::QMC5883LModule(uint8_t id, DroneSystem* ds):
  I2CCompassModule ( id, ds )
 {
   setTypeName(FPSTR(QMC5883L_STR_QMC5883L));
   _sensor = NULL;
   
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = QMC5883L_I2C_ADDRESS;
}


boolean QMC5883LModule::initSensor() {
  if (!_sensor) {
    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
    _sensor = new QMC5883LCompass();
    _sensor->init();
  }
  
  return true;
}


void QMC5883LModule::getSensorValues() {
  if (!_sensor) return;
  
  _sensor->read();

  _raw[0] = _sensor->getX();
  _raw[1] = _sensor->getY();
  _raw[2] = _sensor->getZ();
}
