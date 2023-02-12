#include "INA3221Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "OLEDTomThumbFont.h"

INA3221Module::INA3221Module(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
 {
   setTypeName(FPSTR(INA3221_STR_INA3221));
   _sensor = NULL;
   //_params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = INA3221_I2C_ADDRESS;

   // pubs
   initParams(INA3221_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = INA3221_I2C_ADDRESS;

   DRONE_PARAM_ENTRY *param;

   param = &_params[INA3221_PARAM_CURRENT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, INA3221_PARAM_CURRENT);
   setParamName(FPSTR(STRING_CURRENT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[INA3221_PARAM_POWER_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, INA3221_PARAM_POWER);
   setParamName(FPSTR(STRING_POWER), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[INA3221_PARAM_LOADV_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, INA3221_PARAM_LOADV);
   setParamName(FPSTR(STRING_LOADV), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[INA3221_PARAM_CELLV_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, INA3221_PARAM_CELLV);
   setParamName(FPSTR(STRING_CELLV), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[INA3221_PARAM_ALARM_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, INA3221_PARAM_ALARM);
   setParamName(FPSTR(STRING_ALARM), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 3);
   param->data.uint8[0] = 0;
   param->data.uint8[1] = 0;
   param->data.uint8[2] = 0;

   param = &_params[INA3221_PARAM_CELLS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, INA3221_PARAM_CELLS);
   setParamName(FPSTR(STRING_CELLS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 3);
   param->data.uint8[0] = 3;
   param->data.uint8[1] = 3;
   param->data.uint8[2] = 3;

   param = &_params[INA3221_PARAM_THRESHOLD_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, INA3221_PARAM_THRESHOLD);
   setParamName(FPSTR(STRING_THRESHOLD), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[0] = 11.2;
   param->data.f[1] = 11.2;
   param->data.f[2] = 11.2;
}

INA3221Module::~INA3221Module() {
  if (_sensor) delete _sensor;
}


void INA3221Module::doReset() {
  Log.noticeln("[INA.dR]");
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  if (_sensor) {
    _sensor->reset();
    if (_error) {
      Log.errorln(INA3221_STR_INA3221);
    }
  }
  Log.noticeln("[INA.dR] end");
}


void INA3221Module::setup() {
  I2CBaseModule::setup();
  // instantiate sensor object, now _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] is known
  if (!_sensor) {
    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
    _sensor = new Beastdevices_INA3221(INA3221_ADDR40_GND);
    _sensor->begin();
    _sensor->reset();

    // Set shunt resistors to 100 mOhm for all channels
    _sensor->setShuntRes(100, 100, 100);
  }
}


void INA3221Module::loop() {
  I2CBaseModule::loop();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  // get sensor values
  float current[3], voltage[3];

  current[0] = _sensor->getCurrent(INA3221_CH1);
  current[1] = _sensor->getCurrent(INA3221_CH2);
  current[2] = _sensor->getCurrent(INA3221_CH3);
  updateAndPublishParam(&_params[INA3221_PARAM_CURRENT_E], (uint8_t*)&current, sizeof(current));

  voltage[0] = _sensor->getVoltage(INA3221_CH1);
  voltage[1] = _sensor->getVoltage(INA3221_CH2);
  voltage[2] = _sensor->getVoltage(INA3221_CH3);
  updateAndPublishParam(&_params[INA3221_PARAM_LOADV_E], (uint8_t*)&voltage, sizeof(voltage));

  float power[3];
  for (uint8_t i=0; i<3; i++) {
    power[i] = voltage[i] * current[i];
  }
  updateAndPublishParam(&_params[INA3221_PARAM_POWER_E], (uint8_t*)&power, sizeof(power));

  // calculate cell voltage
  float tempf[3];
  for (uint8_t i=0; i<3; i++) {
    if (_params[INA3221_PARAM_CELLS_E].data.uint8[i] > 0)
      tempf[i] = _params[INA3221_PARAM_LOADV_E].data.f[i] / _params[INA3221_PARAM_CELLS_E].data.uint8[i];
  }
  updateAndPublishParam(&_params[INA3221_PARAM_CELLV_E], (uint8_t*)&tempf, sizeof(tempf));
  /*
  Serial.print("loadV: ");
  Serial.println(tempf);
*/
  // check voltage vs threshold and set alarm
  uint8_t temp8[3];
  for (uint8_t i=0; i<3; i++) {
    temp8[i] = (_params[INA3221_PARAM_LOADV_E].data.f[i] < _params[INA3221_PARAM_THRESHOLD_E].data.f[i]) ? 1 : 0;
  }
  updateAndPublishParam(&_params[INA3221_PARAM_ALARM_E], (uint8_t*)&temp8, sizeof(temp8));
}


uint8_t INA3221Module::diagnosticDisplays() {
  return 1;
}

void INA3221Module::drawDiagnosticDisplay(SSD1306Wire *display, uint8_t page) {
  display->setTextAlignment(TEXT_ALIGN_CENTER);

  uint8_t cw = 42;  // column width

  for (uint8_t i=0; i<3; i++) {
    uint8_t cx = cw * i + cw/2;

    display->setFont(TomThumb4x6);
    display->drawString(cx, 17, String(i));

    display->setFont(ArialMT_Plain_10);
    display->drawString(cx, 27, String(_params[INA3221_PARAM_LOADV_E].data.f[i]) + "v");
    display->drawString(cx, 47, String(_params[INA3221_PARAM_CURRENT_E].data.f[i]) + "a");
  }

}
