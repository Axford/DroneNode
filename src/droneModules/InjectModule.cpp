#include "InjectModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"
#include "DroneSystem.h"

InjectModule::InjectModule(uint8_t id, DroneSystem *ds) : DroneModule(id, ds)
{
    setTypeName(FPSTR(INJECT_STR_INJECT));

    _targetMsg.source(_dlm->node()); // default to local node

    // init subs
    initSubs(INJECT_SUBS);

    DRONE_PARAM_SUB *sub;

    sub = &_subs[INJECT_SUB_INPUT_E];
    sub->addrParam = INJECT_SUB_INPUT_ADDR;
    sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, INJECT_SUB_INPUT);
    setParamName(FPSTR(STRING_INPUT), &sub->param);

    // pubs
    initParams(INJECT_PARAM_ENTRIES);

    DRONE_PARAM_ENTRY *param;

    param = &_params[INJECT_PARAM_TARGET_E];
    param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, INJECT_PARAM_TARGET);
    setParamName(FPSTR(STRING_TARGET), param);
    param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 3);
    param->data.uint8[0] = 0;
    param->data.uint8[1] = 0;
    param->data.uint8[2] = 0;
}

void InjectModule::setup()
{
    DroneModule::setup();

    // configure target message
    if (_params[INJECT_PARAM_TARGET_E].data.uint8[0] != 0)
    {
        _targetMsg.node(_params[INJECT_PARAM_TARGET_E].data.uint8[0]);
        _targetMsg.channel(_params[INJECT_PARAM_TARGET_E].data.uint8[1]);

        _targetMsg._msg.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, _params[INJECT_PARAM_TARGET_E].data.uint8[2]);
        _targetMsg._msg.paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
    } else {
        setError(1);
        disable();
    }
}

void InjectModule::update()
{
    DroneModule::update();

    _targetMsg._msg.payload.f[0] = _subs[INJECT_SUB_INPUT_E].param.data.f[0];
    _dlm->publish(_targetMsg);
}
