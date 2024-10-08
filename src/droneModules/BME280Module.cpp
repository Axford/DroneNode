#include "BME280Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

BME280Module::BME280Module(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
 {
   setTypeName(FPSTR(BME280_STR_BME280));

   _sensor = NULL;

   initParams(BME280_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = BME280_I2C_ADDRESS;

   // init param entries
   _params[BME280_PARAM_TEMPERATURE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, BME280_PARAM_TEMPERATURE);
   _params[BME280_PARAM_TEMPERATURE_E].name = FPSTR(STRING_TEMPERATURE);
   _params[BME280_PARAM_TEMPERATURE_E].nameLen = sizeof(STRING_TEMPERATURE);

   _params[BME280_PARAM_HUMIDITY_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, BME280_PARAM_HUMIDITY);
   _params[BME280_PARAM_HUMIDITY_E].name = FPSTR(STRING_HUMIDITY);
   _params[BME280_PARAM_HUMIDITY_E].nameLen = sizeof(STRING_HUMIDITY);

   _params[BME280_PARAM_PRESSURE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, BME280_PARAM_PRESSURE);
   _params[BME280_PARAM_PRESSURE_E].name = FPSTR(STRING_PRESSURE);
   _params[BME280_PARAM_PRESSURE_E].nameLen = sizeof(STRING_PRESSURE);

   _params[BME280_PARAM_ALTITUDE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, BME280_PARAM_ALTITUDE);
   _params[BME280_PARAM_ALTITUDE_E].name = FPSTR(STRING_ALTITUDE);
   _params[BME280_PARAM_ALTITUDE_E].nameLen = sizeof(STRING_ALTITUDE);
}


void BME280Module::doReset() {
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  setError( _sensor->begin(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]) ? 0 : 1 );
  if (_error) {
    Log.errorln(BME280_STR_BME280);
  }
}


void BME280Module::setup()
{
    I2CBaseModule::setup();
    // instantiate sensor object, now _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] is known
    if (!_sensor)
    {
        DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
        _sensor = new Adafruit_BME280();
        if (!_sensor->begin(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]))
        {
            setError(1);
            disable();
            return;
        }
    }
}


void BME280Module::loop() {
  I2CBaseModule::loop();

  Serial.println("[BME280] .loop");

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  // get sensor values
  _params[BME280_PARAM_TEMPERATURE_E].data.f[0] = _sensor->readTemperature();
  _params[BME280_PARAM_HUMIDITY_E].data.f[0] = _sensor->readHumidity();
  _params[BME280_PARAM_PRESSURE_E].data.f[0] = _sensor->readPressure() / 100.0F;

  // calculate altitude
  // error check
  if (isnan(_params[BME280_PARAM_TEMPERATURE_E].data.f[0])) {
    setError(1);  // will be cleared by next watchdog
  } else {
    float P0 = 1013.25;

    _params[BME280_PARAM_ALTITUDE_E].data.f[0] =
      (pow(P0 / _params[BME280_PARAM_PRESSURE_E].data.f[0], 1/5.257)-1) * (_params[BME280_PARAM_TEMPERATURE_E].data.f[0] + 273.15) / 0.0065;
  }

  // publish param entries
  publishParamEntries();
}
