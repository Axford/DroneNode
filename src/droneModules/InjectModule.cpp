#include "InjectModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../DroneModuleManager.h"
#include "strings.h"
#include "../navMath.h"
#include "DroneSystem.h"

InjectModule::InjectModule(uint8_t id, DroneSystem *ds, Registration registration) : DroneModule(id, ds, registration)
{
    if (!_mgmtParams) return;
    setTypeName(FPSTR(INJECT_STR_INJECT));

    _targetMsg.source(_dlm->node()); // default to local node

    // init subs
    initSubs(INJECT_SUBS);
    if (!_subs) return;

    DRONE_PARAM_SUB *sub;

    sub = &_subs[INJECT_SUB_INPUT_E];
    sub->addrParam = INJECT_SUB_INPUT_ADDR;
    sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, INJECT_SUB_INPUT);
    setParamName(FPSTR(STRING_INPUT), &sub->param);

    // pubs
    initParams(INJECT_PARAM_ENTRIES);
    if (!_params) return;

    DRONE_PARAM_ENTRY *param;

    param = &_params[INJECT_PARAM_TARGET_E];
    param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, INJECT_PARAM_TARGET);
    setParamName(FPSTR(STRING_TARGET), param);
    param->paramTypeLength = _mgmtMsg.packParamLength(registration == Registration::Detached, DRONE_LINK_MSG_TYPE_UINT8_T, 3);
    param->data.uint8[0] = 0;
    param->data.uint8[1] = 0;
    param->data.uint8[2] = 0;

    param = &_params[INJECT_PARAM_MODE_E];
    param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, INJECT_PARAM_MODE);
    setParamName(FPSTR(STRING_MODE), param);
    param->paramTypeLength = _mgmtMsg.packParamLength(registration == Registration::Detached, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
    param->data.uint8[0] = INJECT_MODE_FLOAT;
}

DroneModule* InjectModule::liveSink() const {
    const auto& target = _params[INJECT_PARAM_TARGET_E].data.uint8;
    if (target[0] != _dlm->node() || target[1] == _id) return nullptr;
    auto* sink = _dmm->getModuleById(target[1]);
    if (!sink || !sink->acceptsLiveInjection(target[2])) return nullptr;
    auto* entry = sink->getParamEntryById(target[2]);
    return entry && entry->paramTypeLength == (DRONE_LINK_MSG_WRITABLE | (DRONE_LINK_MSG_TYPE_FLOAT << 4) | 3)
        ? sink : nullptr;
}

bool InjectModule::validateLiveConfiguration() const {
    return _subs[INJECT_SUB_INPUT_E].addr.node &&
           _params[INJECT_PARAM_MODE_E].data.uint8[0] == INJECT_MODE_FLOAT && liveSink();
}

bool InjectModule::dependsOnLiveModule(uint8_t channel) const {
    return _liveLogic && _params[INJECT_PARAM_TARGET_E].data.uint8[1] == channel;
}

void InjectModule::setup()
{
    if (_liveLogic) {
        _params[INJECT_PARAM_TARGET_E].paramTypeLength &= ~DRONE_LINK_MSG_WRITABLE;
        _params[INJECT_PARAM_MODE_E].paramTypeLength &= ~DRONE_LINK_MSG_WRITABLE;
    }
    DroneModule::setup();

    // configure target message
    if (_params[INJECT_PARAM_TARGET_E].data.uint8[0] != 0)
    {
        _targetMsg.node(_params[INJECT_PARAM_TARGET_E].data.uint8[0]);
        _targetMsg.channel(_params[INJECT_PARAM_TARGET_E].data.uint8[1]);

        _targetMsg._msg.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, _params[INJECT_PARAM_TARGET_E].data.uint8[2]);

        switch(_params[INJECT_PARAM_MODE_E].data.uint8[0]) {
        case INJECT_MODE_FLOAT:
            _targetMsg._msg.paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
            break;
        case INJECT_MODE_UINT8_T:
            _targetMsg._msg.paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
            break;
        }
    } else {
        setError(1);
        disable();
    }
}

void InjectModule::update()
{
    DroneModule::update();

    if (_liveLogic) {
        if (auto* sink = liveSink()) {
            _targetMsg._msg.payload.f[0] = _subs[INJECT_SUB_INPUT_E].param.data.f[0];
            sink->handleLinkMessage(&_targetMsg);
        }
        return;
    }

    switch(_params[INJECT_PARAM_MODE_E].data.uint8[0]) {
        case INJECT_MODE_FLOAT:
            _targetMsg._msg.payload.f[0] = _subs[INJECT_SUB_INPUT_E].param.data.f[0];
            break;
        case INJECT_MODE_UINT8_T:
            _targetMsg._msg.payload.uint8[0] = _subs[INJECT_SUB_INPUT_E].param.data.f[0];
            break;
    }
    _dlm->publish(_targetMsg);
}
