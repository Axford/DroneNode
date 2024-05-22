#include "ADS1015Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

#define SS_NEO_PIN 18
#define SS_ENC0_SWITCH 12
#define SS_ENC1_SWITCH 14
#define SS_ENC2_SWITCH 17
#define SS_ENC3_SWITCH 9

ADS1015Module::ADS1015Module(uint8_t id, DroneSystem *ds) : I2CBaseModule(id, ds)
{
    setTypeName(FPSTR(ADS1015_STR_ADS1015));

    _sensor = NULL;

    initParams(ADS1015_PARAM_ENTRIES);

    I2CBaseModule::initBaseParams();
    _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = ADS1015_I2C_ADDRESS;

    // init param entries
    DRONE_PARAM_ENTRY *param;

    // values
    for (uint8_t i=0; i<4; i++) {
        param = &_params[ADS1015_PARAM_VALUE1_E + i];
        param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, ADS1015_PARAM_VALUE1 + i);
        switch(i) {
            case 0: setParamName(FPSTR(STRING_VALUE1), param);  break;
            case 1: setParamName(FPSTR(STRING_VALUE2), param);  break;
            case 2: setParamName(FPSTR(STRING_VALUE3), param);  break;
            case 3: setParamName(FPSTR(STRING_VALUE4), param);  break;
        }
        param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
        param->data.f[0] = 0;
    }

    param = &_params[ADS1015_PARAM_CHANNELS_E];
    param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ADS1015_PARAM_CHANNELS);
    setParamName(FPSTR(STRING_CHANNELS), param);
    param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
    param->data.uint8[0] = 1;

    param = &_params[ADS1015_PARAM_RAW_E];
    param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ADS1015_PARAM_RAW);
    setParamName(FPSTR(STRING_RAW), param);
    param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 16);
    param->data.uint32[0] = 0;
    param->data.uint32[1] = 0;
    param->data.uint32[2] = 0;
    param->data.uint32[3] = 0;

    param = &_params[ADS1015_PARAM_LIMITS_E];
    param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ADS1015_PARAM_LIMITS);
    setParamName(FPSTR(STRING_LIMITS), param);
    param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
    param->data.f[0] = 0;
    param->data.f[1] = 3.3 / 4.096;  // to get a value of 1.0 at 3.3v
}

ADS1015Module::~ADS1015Module()
{
    if (_sensor)
        delete _sensor;
}

void ADS1015Module::doReset()
{
    I2CBaseModule::doReset();

    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

    setError(_sensor->begin(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]) ? 0 : 1);
    if (_error)
    {
        Log.errorln(ADS1015_STR_ADS1015);
    }
}

void ADS1015Module::setup()
{
    I2CBaseModule::setup();
    // instantiate sensor object, now _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] is known
    if (!_sensor)
    {
        DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
        _sensor = new Adafruit_ADS1015();
        if (!_sensor->begin(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]))
        {
            setError(1);
            disable();
            return;
        }

        // configure
        _sensor->setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV

    }
}

void ADS1015Module::loop()
{
    I2CBaseModule::loop();

    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

    uint32_t raw[4] = {0,0,0,0};

    // sample each ADC channel, up to number of channels we're using
    for (int i = 0; i < _params[ADS1015_PARAM_CHANNELS_E].data.uint8[0]; i++)
    {
        //Serial.print("ADC sampling channel: "); Serial.println(i);

        int16_t av;
        av = _sensor->readADC_SingleEnded(i);
        raw[i] = av;

        float f = _params[ADS1015_PARAM_LIMITS_E].data.f[0] + (_params[ADS1015_PARAM_LIMITS_E].data.f[1]-_params[ADS1015_PARAM_LIMITS_E].data.f[0]) * (av/4095.0);

        updateAndPublishParam(&_params[ADS1015_PARAM_VALUE1_E + i], (uint8_t*)&f, sizeof(f));
    }

    updateAndPublishParam(&_params[ADS1015_PARAM_RAW_E], (uint8_t*)&raw, 16);
}
