#include "INA219Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

INA219Module::INA219Module(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  I2CBaseModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(INA219_STR_INA219));
   _addr = INA219_I2C_ADDRESS;

   _numCells = 1;
   _threshold = 0;  //voltage threshold for alarm

   // pubs
   initParams(INA219_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[INA219_PARAM_SHUNTV_E];
   param->param = INA219_PARAM_SHUNTV;
   setParamName(FPSTR(STRING_SHUNTV), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[INA219_PARAM_BUSV_E];
   param->param = INA219_PARAM_BUSV;
   setParamName(FPSTR(STRING_BUSV), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[INA219_PARAM_CURRENT_E];
   param->param = INA219_PARAM_CURRENT;
   setParamName(FPSTR(STRING_CURRENT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[INA219_PARAM_POWER_E];
   param->param = INA219_PARAM_POWER;
   setParamName(FPSTR(STRING_POWER), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[INA219_PARAM_LOADV_E];
   param->param = INA219_PARAM_LOADV;
   setParamName(FPSTR(STRING_LOADV), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[INA219_PARAM_CELLV_E];
   param->param = INA219_PARAM_CELLV;
   setParamName(FPSTR(STRING_CELLV), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[INA219_PARAM_ALARM_E];
   param->param = INA219_PARAM_ALARM;
   setParamName(FPSTR(STRING_ALARM), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);

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

  _numCells = obj[STRING_CELLS] | _numCells;
  if (_numCells < 1) _numCells = 1;

  _threshold = obj[STRING_THRESHOLD] | _threshold;
}



void INA219Module::loop() {
  I2CBaseModule::loop();

  DroneWire::selectChannel(_bus);

  // get sensor values
  float tempf;

  tempf = _sensor->getShuntVoltage_mV() / 1000.0f;
  updateAndPublishParam(&_params[INA219_PARAM_SHUNTV_E], (uint8_t*)&tempf, sizeof(tempf));

  tempf = _sensor->getBusVoltage_V();
  updateAndPublishParam(&_params[INA219_PARAM_BUSV_E], (uint8_t*)&tempf, sizeof(tempf));

  tempf = _sensor->getCurrent_mA() / 1000.0f;
  updateAndPublishParam(&_params[INA219_PARAM_CURRENT_E], (uint8_t*)&tempf, sizeof(tempf));

  tempf = _sensor->getPower_mW() / 1000.0f;
  updateAndPublishParam(&_params[INA219_PARAM_POWER_E], (uint8_t*)&tempf, sizeof(tempf));

  tempf = _params[INA219_PARAM_BUSV_E].data.f[0] + _params[INA219_PARAM_SHUNTV_E].data.f[0];
  updateAndPublishParam(&_params[INA219_PARAM_LOADV_E], (uint8_t*)&tempf, sizeof(tempf));

  // calculate cell voltage
  tempf = _params[INA219_PARAM_LOADV_E].data.f[0] / _numCells;
  updateAndPublishParam(&_params[INA219_PARAM_CELLV_E], (uint8_t*)&tempf, sizeof(tempf));

  // check voltage vs threshold and set alarm
  uint8_t temp8 = (_params[INA219_PARAM_LOADV_E].data.f[0] < _threshold) ? 1 : 0;
  updateAndPublishParam(&_params[INA219_PARAM_ALARM_E], (uint8_t*)&temp8, sizeof(temp8));

  // error check
  if (isnan(_params[INA219_PARAM_SHUNTV_E].data.f[0])) {
    setError(1);  // will be cleared by next watchdog
  }

  // publish param entries
  //publishParamEntries();


}
