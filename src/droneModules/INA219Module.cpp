#include "INA219Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

INA219Module::INA219Module(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  I2CBaseModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(INA219_STR_INA219));
   _sensor = NULL;
   //_params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = INA219_I2C_ADDRESS;

   // pubs
   initParams(INA219_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = INA219_I2C_ADDRESS;

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

   param = &_params[INA219_PARAM_CELLS_E];
   param->param = INA219_PARAM_CELLS;
   setParamName(FPSTR(STRING_CELLS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[INA219_PARAM_CELLS_E].data.uint8[0] = 3;

   param = &_params[INA219_PARAM_THRESHOLD_E];
   param->param = INA219_PARAM_THRESHOLD;
   setParamName(FPSTR(STRING_THRESHOLD), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[INA219_PARAM_THRESHOLD_E].data.f[0] = 11.2;
}

INA219Module::~INA219Module() {
  if (_sensor) delete _sensor;
}


DEM_NAMESPACE* INA219Module::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(INA219_STR_INA219,0,true);
}

void INA219Module::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  I2CBaseModule::registerParams(ns, dem);

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_THRESHOLD, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, STRING_CELLS, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
}


void INA219Module::doReset() {
  Log.noticeln("[INA.dR]");
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  if (_sensor) {
    setError( _sensor->begin() ? 1 : 0 );
    if (_error) {
      Log.errorln(INA219_STR_INA219);
    }
  }
  Log.noticeln("[INA.dR] end");
}


void INA219Module::setup() {
  I2CBaseModule::setup();
  // instantiate sensor object, now _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] is known
  if (!_sensor) {
    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
    _sensor = new Adafruit_INA219(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]);
    _sensor->begin();
  }
}


void INA219Module::loop() {
  I2CBaseModule::loop();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

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
  tempf = _params[INA219_PARAM_LOADV_E].data.f[0] / _params[INA219_PARAM_CELLS_E].data.uint8[0];
  updateAndPublishParam(&_params[INA219_PARAM_CELLV_E], (uint8_t*)&tempf, sizeof(tempf));
  /*
  Serial.print("loadV: ");
  Serial.println(tempf);
*/
  // check voltage vs threshold and set alarm
  uint8_t temp8 = (_params[INA219_PARAM_LOADV_E].data.f[0] < _params[INA219_PARAM_THRESHOLD_E].data.f[0]) ? 1 : 0;
  updateAndPublishParam(&_params[INA219_PARAM_ALARM_E], (uint8_t*)&temp8, sizeof(temp8));

  // error check
  if (isnan(_params[INA219_PARAM_LOADV_E].data.f[0])) {
    setError(1);  // will be cleared by next watchdog
  }

  // publish param entries
  //publishParamEntries();


}
