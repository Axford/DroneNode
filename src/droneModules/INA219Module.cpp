#include "INA219Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

// @type INA219

INA219Module::INA219Module(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
 {
   setTypeName(FPSTR(INA219_STR_INA219));
   _sensor = NULL;
   _lastLoopTime = 0;
   //_params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = INA219_I2C_ADDRESS;

   // pubs
   initParams(INA219_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = INA219_I2C_ADDRESS;

   DRONE_PARAM_ENTRY *param;

   param = &_params[INA219_PARAM_SHUNTV_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, INA219_PARAM_SHUNTV);
   setParamName(FPSTR(STRING_SHUNTV), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[INA219_PARAM_BUSV_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, INA219_PARAM_BUSV);
   setParamName(FPSTR(STRING_BUSV), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[INA219_PARAM_CURRENT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, INA219_PARAM_CURRENT);
   setParamName(FPSTR(STRING_CURRENT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[INA219_PARAM_POWER_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, INA219_PARAM_POWER);
   setParamName(FPSTR(STRING_POWER), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[INA219_PARAM_LOADV_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, INA219_PARAM_LOADV);
   setParamName(FPSTR(STRING_LOADV), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[INA219_PARAM_CELLV_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, INA219_PARAM_CELLV);
   setParamName(FPSTR(STRING_CELLV), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[INA219_PARAM_ALARM_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, INA219_PARAM_ALARM);
   setParamName(FPSTR(STRING_ALARM), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);

   param = &_params[INA219_PARAM_CELLS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, INA219_PARAM_CELLS);
   setParamName(FPSTR(STRING_CELLS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
// @default cells=3
   _params[INA219_PARAM_CELLS_E].data.uint8[0] = 3;

   param = &_params[INA219_PARAM_THRESHOLD_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, INA219_PARAM_THRESHOLD);
   setParamName(FPSTR(STRING_THRESHOLD), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
// @default threshold=11.2
   _params[INA219_PARAM_THRESHOLD_E].data.f[0] = 11.2;

   param = &_params[INA219_PARAM_USAGE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, INA219_PARAM_USAGE);
   setParamName(FPSTR(STRING_USAGE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   param->data.f[0] = 0;
}

INA219Module::~INA219Module() {
  if (_sensor) delete _sensor;
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
  _lastLoopTime = millis();
}


void INA219Module::loop() {
  I2CBaseModule::loop();

  unsigned long loopTime = millis();
  float dt = (loopTime - _lastLoopTime) / 1000.0; // in seconds
  if (dt < 0.001) dt = 0.001;
  if (dt > 100) dt = 1;  // in case of overflow
  _lastLoopTime = loopTime;

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  // get sensor values
  float tempf;

  tempf = _sensor->getShuntVoltage_mV() / 1000.0f;
  updateAndPublishParam(&_params[INA219_PARAM_SHUNTV_E], (uint8_t*)&tempf, sizeof(tempf));

  tempf = _sensor->getBusVoltage_V();
  updateAndPublishParam(&_params[INA219_PARAM_BUSV_E], (uint8_t*)&tempf, sizeof(tempf));

  tempf = _sensor->getCurrent_mA() / 1000.0f;
  updateAndPublishParam(&_params[INA219_PARAM_CURRENT_E], (uint8_t*)&tempf, sizeof(tempf));

  tempf = _params[INA219_PARAM_USAGE_E].data.f[0] + (tempf * dt/3600);
  updateAndPublishParam(&_params[INA219_PARAM_USAGE_E], (uint8_t*)&tempf, sizeof(tempf));

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
