#include "I2CBaseModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

I2CBaseModule::I2CBaseModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;
}

void I2CBaseModule::initBaseParams() {
  DRONE_PARAM_ENTRY *param;

  param = &_params[I2CBASE_PARAM_BUS_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, I2CBASE_PARAM_BUS);
  setParamName(FPSTR(STRING_BUS), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
  _params[I2CBASE_PARAM_BUS_E].data.uint8[0] = 0;

  param = &_params[I2CBASE_PARAM_ADDR_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, I2CBASE_PARAM_ADDR);
  setParamName(FPSTR(STRING_ADDR), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
  _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = 0;
}


void I2CBaseModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_BUS, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_ADDR, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
}


void I2CBaseModule::doReset() {
  if (!_setupDone) return;

  Log.warningln(F("[I2C.dR]"));
  if (_resetCount > 1) {
    // attempt resetting the multiplexer
    DroneWire::reset();

  }
  _resetCount++;
}


boolean I2CBaseModule::isAlive() {
  // if already in error state, then assume dead
  //if (_error > 0) return false;
  // poll sensor to see if we're alive
  if (!_setupDone || _params[I2CBASE_PARAM_ADDR_E].data.uint8[0]==0) return true; // avoid false resets

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  Wire.beginTransmission(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]);
  byte err = Wire.endTransmission();
  if (err != 0) {
    setError(1);
  } else {
    setError(0);
  }
  return (err == 0);
}


void I2CBaseModule::setup() {
  DroneModule::setup();

  //doReset();
}
