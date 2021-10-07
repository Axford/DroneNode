#include "INA219Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"


INA219Module::INA219Module(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  I2CBaseModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(INA219_STR_INA219));
   _addr = INA219_I2C_ADDRESS;

   _numParamEntries = INA219_PARAM_ENTRIES;
   _params = new DRONE_PARAM_ENTRY[_numParamEntries];

   // defaults
   for (uint8_t i=0; i<_numParamEntries; i++) {
     _params[i].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
     _params[i].publish = false;
     _params[i].data.f[0] = 0;
   }

   _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, sizeof(INA219_STR_INA219));
   strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, INA219_STR_INA219, sizeof(INA219_STR_INA219));


   // init param entries
   _params[INA219_PARAM_SHUNTV_E].param = INA219_PARAM_SHUNTV;
   _params[INA219_PARAM_SHUNTV_E].name = FPSTR(DRONE_STR_SHUNTV);
   _params[INA219_PARAM_SHUNTV_E].nameLen = sizeof(DRONE_STR_SHUNTV);


   _params[INA219_PARAM_BUSV_E].param = INA219_PARAM_BUSV;
   _params[INA219_PARAM_BUSV_E].name = FPSTR(DRONE_STR_BUSV);
   _params[INA219_PARAM_BUSV_E].nameLen = sizeof(DRONE_STR_BUSV);


   _params[INA219_PARAM_CURRENT_E].param = INA219_PARAM_CURRENT;
   _params[INA219_PARAM_CURRENT_E].name = FPSTR(DRONE_STR_CURRENT);
   _params[INA219_PARAM_CURRENT_E].nameLen = sizeof(DRONE_STR_CURRENT);


   _params[INA219_PARAM_POWER_E].param = INA219_PARAM_POWER;
   _params[INA219_PARAM_POWER_E].name = FPSTR(DRONE_STR_POWER);
   _params[INA219_PARAM_POWER_E].nameLen = sizeof(DRONE_STR_POWER);


   _params[INA219_PARAM_LOADV_E].param = INA219_PARAM_LOADV;
   _params[INA219_PARAM_LOADV_E].name = FPSTR(DRONE_STR_LOADV);
   _params[INA219_PARAM_LOADV_E].nameLen = sizeof(DRONE_STR_LOADV);

}

INA219Module::~INA219Module() {
  if (_sensor) delete _sensor;
}


void INA219Module::doReset() {
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_bus);


  setError( _sensor->begin() ? 0 : 1 );
  if (_error) {
    Log.errorln(INA219_STR_INA219);
  }

}


void INA219Module::loadConfiguration(JsonObject &obj) {
  I2CBaseModule::loadConfiguration(obj);

  // instantiate sensor object, now _addr is known
  _sensor = new Adafruit_INA219(_addr);
}


void INA219Module::loop() {
  I2CBaseModule::loop();

  DroneWire::selectChannel(_bus);

  // get sensor values
  _params[INA219_PARAM_SHUNTV_E].data.f[0] = _sensor->getShuntVoltage_mV() / 1000.0f;
  _params[INA219_PARAM_BUSV_E].data.f[0] = _sensor->getBusVoltage_V();
  _params[INA219_PARAM_CURRENT_E].data.f[0] = _sensor->getCurrent_mA() / 1000.0f;
  _params[INA219_PARAM_POWER_E].data.f[0] = _sensor->getPower_mW() / 1000.0f;
  _params[INA219_PARAM_LOADV_E].data.f[0] = _params[INA219_PARAM_BUSV_E].data.f[0] + _params[INA219_PARAM_SHUNTV_E].data.f[0];

  // error check
  if (isnan(_params[INA219_PARAM_SHUNTV_E].data.f[0])) {
    setError(1);  // will be cleared by next watchdog
  }

  // publish param entries
  publishParamEntries();


}
