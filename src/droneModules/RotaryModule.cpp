#include "RotaryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

#define SS_NEO_PIN 18
#define SS_ENC0_SWITCH 12
#define SS_ENC1_SWITCH 14
#define SS_ENC2_SWITCH 17
#define SS_ENC3_SWITCH 9

RotaryModule::RotaryModule(uint8_t id, DroneSystem *ds) : I2CBaseModule(id, ds)
{
    setTypeName(FPSTR(ROTARY_STR_ROTARY));

    _encPositions[0] = 0;
    _encPositions[1] = 0;
    _encPositions[2] = 0;
    _encPositions[3] = 0;

    initParams(ROTARY_PARAM_ENTRIES);

    I2CBaseModule::initBaseParams();
    _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = ROTARY_I2C_ADDRESS;

    // init param entries
    DRONE_PARAM_ENTRY *param;

    // inputs
    for (uint8_t i=0; i<4; i++) {
        param = &_params[ROTARY_PARAM_INPUT1_E + i];
        param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, ROTARY_PARAM_INPUT1 + i);
        switch(i) {
            case 0: setParamName(FPSTR(STRING_INPUT1), param);  break;
            case 1: setParamName(FPSTR(STRING_INPUT2), param);  break;
            case 2: setParamName(FPSTR(STRING_INPUT3), param);  break;
            case 3: setParamName(FPSTR(STRING_INPUT4), param);  break;
        }
        param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
        param->data.f[0] = 0;
    }

    param = &_params[ROTARY_PARAM_MAP_E];
    param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ROTARY_PARAM_MAP);
    setParamName(FPSTR(STRING_MAP), param);
    param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 16);
    param->data.f[0] = 1;
    param->data.f[1] = 1;
    param->data.f[2] = 1;
    param->data.f[3] = 1;
}

RotaryModule::~RotaryModule()
{
    if (_sensor)
        delete _sensor;
}

void RotaryModule::doReset()
{
    I2CBaseModule::doReset();

    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

    setError(_sensor->begin(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]) ? 0 : 1);
    if (_error)
    {
        Log.errorln(ROTARY_STR_ROTARY);
    }
}

void RotaryModule::setup()
{
    I2CBaseModule::setup();
    // instantiate sensor object, now _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] is known
    if (!_sensor)
    {
        DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
        _sensor = new Adafruit_seesaw(&Wire);
        if (!_sensor->begin(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]))
        {
            setError(1);
            disable();
            return;
        }

        // configure
        _sensor->pinMode(SS_ENC0_SWITCH, INPUT_PULLUP);
        _sensor->pinMode(SS_ENC1_SWITCH, INPUT_PULLUP);
        _sensor->pinMode(SS_ENC2_SWITCH, INPUT_PULLUP);
        _sensor->pinMode(SS_ENC3_SWITCH, INPUT_PULLUP);
        _sensor->setGPIOInterrupts(1UL << SS_ENC0_SWITCH | 1UL << SS_ENC1_SWITCH |
                                       1UL << SS_ENC2_SWITCH | 1UL << SS_ENC3_SWITCH,
                                   1);

        // set starting positions
        for (int e = 0; e < 4; e++)
        {
            /*
            // Firmware issue means setting encoder positions does not work!! 
            https://github.com/adafruit/Adafruit_Seesaw/issues/90

            _encPositions[e] = floor(_params[ROTARY_PARAM_INPUT1_E + e].data.f[0] / _params[ROTARY_PARAM_MAP_E].data.f[e]);

            _sensor->setEncoderPosition(_encPositions[e], e);
            */

            // as a workaround, we'll store offsets based on the user configured default values
            // this will be added back to read values to get a final output
            _encOffsets[e] = floor(_params[ROTARY_PARAM_INPUT1_E + e].data.f[0] / _params[ROTARY_PARAM_MAP_E].data.f[e]);

            _encPositions[e] = 0;  // always inits to zero

            //_sensor->enableEncoderInterrupt(e);
        }
    }
}

void RotaryModule::loop()
{
    I2CBaseModule::loop();

    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

    // sample each encoder
    for (int e = 0; e < 4; e++)
    {
        int32_t newPos = _sensor->getEncoderPosition(e) + _encOffsets[e];
        // did we move around?
        if (_encPositions[e] != newPos)
        {
            _encPositions[e] = newPos;
            
            // convert to output range and publish
            float f = newPos * _params[ROTARY_PARAM_MAP_E].data.f[e];

            updateAndPublishParam(&_params[ROTARY_PARAM_INPUT1_E + e], (uint8_t*)&f, sizeof(f));
        }
    }
}
