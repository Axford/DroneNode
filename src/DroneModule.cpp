#include "DroneModule.h"
#include "DroneLinkMsg.h"
#include "DroneLinkManager.h"
#include "DroneModuleManager.h"
#include "pinConfig.h"

// PIN Name to Number lookups

#define PINS_NUM    10

char PIN_NAMES[PINS_NUM] [7] = {
  { "OUT0_0" },
  { "OUT0_1" },
  { "OUT1_0" },
  { "OUT1_1" },
  { "OUT2_0" },
  { "OUT2_1" },
  { "DAC0_0" },
  { "DAC0_1" },
  { "IN0_0" },
  { "IN0_1" }
};

uint8_t PIN_NUMBERS[PINS_NUM] = {
  PIN_OUT0_0,
  PIN_OUT0_1,
  PIN_OUT1_0,
  PIN_OUT1_1,
  PIN_OUT2_0,
  PIN_OUT2_1,
  PIN_DAC0_0,
  PIN_DAC0_1,
  PIN_IN0_0,
  PIN_IN0_1
};


DroneModule::DroneModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
_dlm(dlm),
_dmm(dmm),
_id(id) {
  _dmm->registerModule(this);
  _enabled = true;
  _error = 0;
  _dlm->subscribe(id, this, DRONE_LINK_PARAM_ALL); // subscribe to all params for self
  _mgmtMsg.source(_dlm->node()); // default to local node
  _mgmtMsg.node(_dlm->node()); // default to local node
  _mgmtMsg.channel(_id); // pre-set mgmtmsg with own id
  _resetCount = 0;
  _lastReset = 0;
  _numParamEntries = 0;
  _numSubs = 0;
  _loopInterval = 0;  // default no delay between loop calls
  _lastLoop = 0;

  // alloc for mgmt params
  _mgmtParams = (DRONE_PARAM_ENTRY*)malloc( sizeof(DRONE_PARAM_ENTRY) * DRONE_MGMT_PARAM_ENTRIES);

  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].param = DRONE_MODULE_PARAM_STATUS;
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].name = FPSTR(DRONE_STR_STATUS);
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].nameLen = sizeof(DRONE_STR_STATUS);
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].publish = true;
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].data.uint8[0] = _enabled ? 1 : 0;

  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].param = DRONE_MODULE_PARAM_NAME;
  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].name = FPSTR(DRONE_STR_NAME);
  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].nameLen = sizeof(DRONE_STR_NAME);
  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, 1);
  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].publish = true;
  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].data.c[0] = '?';

  _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].param = DRONE_MODULE_PARAM_ERROR;
  _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].name = FPSTR(DRONE_STR_ERROR);
  _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].nameLen = sizeof(DRONE_STR_ERROR);
  _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
  _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].publish = true;
  _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].data.uint8[0] = _error;

  _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].param = DRONE_MODULE_PARAM_RESETCOUNT;
  _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].name = FPSTR(DRONE_STR_RESETCOUNT);
  _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].nameLen = sizeof(DRONE_STR_RESETCOUNT);
  _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
  _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].publish = true;
  _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].data.uint8[0] = _resetCount;

  _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].param = DRONE_MODULE_PARAM_TYPE;
  _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].name = FPSTR(DRONE_STR_TYPE);
  _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].nameLen = sizeof(DRONE_STR_TYPE);
  _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].publish = true;
  _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, sizeof(DRONE_STR_DRONE));
  strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, DRONE_STR_DRONE, sizeof(DRONE_STR_DRONE));

}


DroneModule::~DroneModule() { }


void DroneModule::setTypeName( const __FlashStringHelper * name) {
  int len = strlen_P((PGM_P)name);
  _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, len);
  strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, (PGM_P)name, len);
}


void DroneModule::setParamName( const __FlashStringHelper * name, DRONE_PARAM_ENTRY *param) {
  param->name = name;
  param->nameLen = strlen_P((PGM_P)name);
}


void DroneModule::initSubs(uint8_t numSubs) {
  _numSubs = numSubs;
  _subs = (DRONE_PARAM_SUB*)malloc( sizeof(DRONE_PARAM_SUB) * _numSubs);

  for (uint8_t i=0; i<_numSubs; i++) {
    _subs[i].received = false;
    _subs[i].addr.param = 255;
    _subs[i].param.publish = false;
    _subs[i].param.nameLen = sizeof(DRONE_STR_BLANK);
    _subs[i].param.name = FPSTR(DRONE_STR_BLANK);
    // input values are writable by default, unless wired to a valid address
    _subs[i].param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
    _subs[i].param.data.f[0] = 0;
  }
}

void DroneModule::initParams(uint8_t numParams) {
  _numParamEntries = numParams;
  _params = (DRONE_PARAM_ENTRY*)malloc( sizeof(DRONE_PARAM_ENTRY) * numParams);

  for (uint8_t i=0; i<_numParamEntries; i++) {
    _params[i].publish = false;
    _params[i].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
    _params[i].data.f[0] = 0;
  }
}


void DroneModule::reset() {
  unsigned long loopTime = millis();
  if (loopTime > _lastReset + DRONE_MODULE_RESET_INTERVAL) {
    Log.warningln(F("Resetting %d"), _id);
    _lastReset = loopTime;
    _resetCount++;

    // publish new reset count
    _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].data.uint8[0] = _resetCount;
    publishParamEntry(&_mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E]);

    doReset();
  }
}

void DroneModule::doReset() {
  // override this
}

void DroneModule::onOTAProgress(float progress) {
  // override this
}

void DroneModule::doShutdown() {
  _enabled = false;
}


boolean DroneModule::isAlive() { return true; }


void DroneModule::parsePins(JsonObject &obj, uint8_t *pins, uint8_t numPins) {
  if (obj.containsKey(DRONE_STR_PINS)) {
    Log.noticeln(DRONE_STR_PINS);

    if (obj[DRONE_STR_PINS].is<JsonArray>()) {
      JsonArray array = obj[DRONE_STR_PINS].as<JsonArray>();

      uint8_t i = 0;
      for(JsonVariant v : array) {
        if (i<numPins) {
          String pinName = v;
          // attempt to lookup pin number from name
          for (uint8_t j=0; j<PINS_NUM; j++) {
            if (pinName == PIN_NAMES[j]) {
              pins[i] = PIN_NUMBERS[j];
              break;
            }
          }
        }
        i++;
      }
    } else {
      // attempt to lookup pin number from name
      String pinName = obj[DRONE_STR_PINS];

      for (uint8_t j=0; j<PINS_NUM; j++) {
        if (pinName == PIN_NAMES[j]) {
          pins[0] = PIN_NUMBERS[j];
          break;
        }
      }
    }

  }
}


void DroneModule::loadConfiguration(JsonObject &obj) {
  String name = obj[DRONE_STR_NAME] | "unnamed";
  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, name.length());
  name.toCharArray(_mgmtParams[DRONE_MODULE_PARAM_NAME_E].data.c, 16);
  Log.noticeln(F("[DroneModule.loadConfiguration] loading config for: %s"), name.c_str());

  _enabled = obj[DRONE_STR_ENABLE] | _enabled;
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].data.uint8[0] = _enabled ? 1 : 0;

  _loopInterval = obj[DRONE_STR_INTERVAL] | _loopInterval;

  // subs: []  ... used for telemetry
  if (obj.containsKey(DRONE_STR_SUBSCRIBETO)) {
    Log.noticeln(F("[DroneModule.loadConfiguration]  Read subs..."));
    JsonArray array = obj[DRONE_STR_SUBSCRIBETO].as<JsonArray>();
    DRONE_LINK_ADDR addr;
    for(JsonVariant v : array) {
      addr = DroneLinkMsg::parseAddress(v);
      _dlm->subscribe(addr.node, addr.channel, this, addr.param);
    }
  }

  // read input addresses
  Log.noticeln(F("[DroneModule.loadConfiguration] Read inputs..."));
  for (uint8_t i=0; i<_numSubs; i++) {
    if (obj.containsKey(_subs[i].param.name)) {
      Log.noticeln(F("  Name: %s"),_subs[i].param.name);

      JsonArray array = obj[_subs[i].param.name].as<JsonArray>();

      // element 0 should contain the address
      _subs[i].addr = DroneLinkMsg::parseAddress( array[0] );
      Log.noticeln(F("  Addr: %d>%d.%d"), _subs[i].addr.node, _subs[i].addr.channel, _subs[i].addr.param);
      if (_subs[i].addr.param != 255) {
        // make param value read-only
        _subs[i].param.paramTypeLength &= ~DRONE_LINK_MSG_WRITABLE;
      }

      // element 1 should contain an array of default values

      if (array.size() == 2) {
        JsonArray defaults = array[1].as<JsonArray>();
        uint8_t j=0;
        for(JsonVariant v : defaults) {
          if (j < (_subs[i].param.paramTypeLength & 0xF)+1) {
            _subs[i].received = true;
            switch((_subs[i].param.paramTypeLength >> 4) & 0x7) {
              case DRONE_LINK_MSG_TYPE_UINT8_T:
                   _subs[i].param.data.uint8[j] = v | 0;
                   break;
              case DRONE_LINK_MSG_TYPE_UINT32_T:
                   _subs[i].param.data.uint32[j] = v | 0;
                   break;
              case DRONE_LINK_MSG_TYPE_FLOAT:
                   _subs[i].param.data.f[j] = v | 0.0;
                   break;
            }
          }
          j++;
        }
      }

    }
  }


  // read publish settings
  if (obj.containsKey(DRONE_STR_PUBLISH)) {
    Log.noticeln(F("[DroneModule.loadConfiguration] Read publish settings..."));
    JsonArray array = obj[DRONE_STR_PUBLISH].as<JsonArray>();
    for(JsonVariant v : array) {
      // see if this matches an output param
      for (uint8_t i=0; i<_numParamEntries; i++) {
        if (_params[i].name == v)
          _params[i].publish = true;
      }

      // or a sub
      for (uint8_t i=0; i<_numSubs; i++) {
        if (_subs[i].param.name == v)
          _subs[i].param.publish = true;
      }

    }
  }

  Log.noticeln(F("[DroneModule.loadConfiguration] ok"));
}

boolean DroneModule::publishParamEntry(DRONE_PARAM_ENTRY *param) {
  _mgmtMsg._msg.param = param->param;
  _mgmtMsg._msg.paramTypeLength = param->paramTypeLength;
  memcpy(_mgmtMsg._msg.payload.c, param->data.c, 16);
  return _dlm->publish(_mgmtMsg);
}

boolean DroneModule::publishMgmtParamEntries() {
  boolean res = true;
  for (uint8_t i=0; i<DRONE_MGMT_PARAM_ENTRIES; i++) {
    if (_mgmtParams[i].publish) {
      res = res && publishParamEntry(&_mgmtParams[i]);
    }
  }
  return res;
}

boolean DroneModule::publishParamEntries() {
  boolean res = true;
  for (uint8_t i=0; i<_numParamEntries; i++) {
    if (_params[i].publish) {
      res = res && publishParamEntry(&_params[i]);
    }
  }
  return res;
}

boolean DroneModule::publishSubAddress(DRONE_PARAM_SUB *sub) {
  _mgmtMsg._msg.param = sub->addrParam;
  _mgmtMsg._msg.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_ADDR, sizeof(DRONE_LINK_ADDR));
  memcpy(_mgmtMsg._msg.payload.c, (uint8_t*)&sub->addr, sizeof(DRONE_LINK_ADDR));
  return _dlm->publish(_mgmtMsg);
}

boolean DroneModule::publishSubs() {
  boolean res = true;
  for (uint8_t i=0; i<_numSubs; i++) {
    if (_subs[i].param.publish) {
      res = res && publishParamEntry(&_subs[i].param);

      // also publish the address "virtual" param
      res = res && publishSubAddress(&_subs[i]);
    }
  }
  return res;
}


void DroneModule::onParamWrite(DRONE_PARAM_ENTRY *param) {
  if (param->param == DRONE_MODULE_PARAM_STATUS) {
    switch(param->data.uint8[0]) {
      case 0: disable();
        break;
      case 1: enable();
        break;
    }
  }
}


void DroneModule::handleParamMessage(DroneLinkMsg *msg, DRONE_PARAM_ENTRY *param) {
  if (msg->type() == ((param->paramTypeLength >> 4) & 0x07)) {
    // write param
    if ((param->paramTypeLength & DRONE_LINK_MSG_WRITABLE) > 0) {
      uint8_t len = msg->length();
      // compare to see if anything has changed... which may including receiving our own messages after a query!!
      if (memcmp(param->data.c, msg->_msg.payload.c, len) != 0) {
        Log.noticeln("Write to param: ");
        msg->print();
        memcpy(param->data.c, msg->_msg.payload.c, len);
        // trigger callback
        onParamWrite(param);
        update();
      }
    }

  } else if (msg->type() == DRONE_LINK_MSG_TYPE_QUERY) {
    // query -> publish
    publishParamEntry(param);

  } else if (msg->type() == DRONE_LINK_MSG_TYPE_NAMEQUERY) {
    // query name
    _mgmtMsg._msg.param = param->param;
    _mgmtMsg.type(DRONE_LINK_MSG_TYPE_NAME);
    _mgmtMsg.length(param->nameLen);
    memcpy(_mgmtMsg._msg.payload.c, param->name, param->nameLen);
    _dlm->publish(_mgmtMsg);
  }
}

void DroneModule::handleSubAddrMessage(DroneLinkMsg *msg, DRONE_PARAM_SUB *sub) {
  if (msg->type() == DRONE_LINK_MSG_TYPE_ADDR) {
    // write new address
    uint8_t len = msg->length();  // should always be sizeof(DRONE_LINK_ADDR) //msg->length();
    // compare to see if anything has changed... which may including receiving our own messages after a query!!
    if (memcmp((uint8_t*)&sub->addr, msg->_msg.payload.c, len) != 0) {
      Log.noticeln("Write to address: ");
      msg->print();
      memcpy((uint8_t*)&sub->addr, msg->_msg.payload.c, len);
      // trigger callback
      // TODO: decide if we need callbacks for this?
      //onParamWrite(param);
    }


  } else if (msg->type() == DRONE_LINK_MSG_TYPE_QUERY) {
    // query -> publish
    publishSubAddress(sub);

  } else if (msg->type() == DRONE_LINK_MSG_TYPE_NAMEQUERY) {
    // query name
    _mgmtMsg._msg.param = sub->addrParam;
    _mgmtMsg.type(DRONE_LINK_MSG_TYPE_NAME);
    _mgmtMsg.length(sub->param.nameLen);
    memcpy(_mgmtMsg._msg.payload.c, sub->param.name, sub->param.nameLen);
    _dlm->publish(_mgmtMsg);
  }
}


boolean DroneModule::handleManagementMessage(DroneLinkMsg *msg) {
  if (msg->param() < DRONE_CUSTOM_PARAM_START) {
    for (uint8_t i=0; i<DRONE_MGMT_PARAM_ENTRIES; i++) {
      if (msg->param() == _mgmtParams[i].param) {
        handleParamMessage(msg, &_mgmtParams[i]);
      }
    }
    return true;
  }
  return false;
}


void DroneModule::handleLinkMessage(DroneLinkMsg *msg) {

  // handle subs
  for (uint8_t i=0; i<_numSubs; i++) {
    if (msg->sameAddress(&_subs[i].addr)) {

      // check the type matches what's expected
      if (msg->type() == ((_subs[i].param.paramTypeLength >> 4) & 0x7)) {
        // handle the subscription updating
        memcpy(_subs[i].param.data.c, msg->_msg.payload.c, msg->length());

        _subs[i].received = true;

        // publish the change to this param
        if (_subs[i].param.publish) publishParamEntry(&_subs[i].param);

        // trigger an update
        update();
      }
    }
  }

  if (msg->channel() != _id) return;

  if (!handleManagementMessage(msg)) {

    if (_enabled) {
      // handle output params
      for (uint8_t i=0; i<_numParamEntries; i++) {
        if (msg->param() == _params[i].param) {
          handleParamMessage(msg, &_params[i]);
        }
      }

      // handle subs
      for (uint8_t i=0; i<_numSubs; i++) {
        if (msg->param() == _subs[i].param.param) {
          // handles writes to the internal value
          handleParamMessage(msg, &_subs[i].param);
        } else if (msg->param() == _subs[i].addrParam) {
          // handle writes to the address
          handleSubAddrMessage(msg, &_subs[i]);
        }
      }
    }
  }
}

void DroneModule::setError(uint8_t error) {
  if (error != _error) {
    _error = error;

    Log.errorln(F("Error state change: %d=%d"), _id, _error);

    // publish
    _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].data.uint8[0] = _error;
    publishParamEntry(&_mgmtParams[DRONE_MODULE_PARAM_ERROR_E]);
  }
}

void DroneModule::enable() {
  if (_enabled) return;
  _enabled = true;
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].data.uint8[0] = _enabled ? 1 : 0;
  Log.noticeln(F("Enable module %d"), _id);
}

void DroneModule::disable() {
  if (!_enabled) return;
  _enabled = false;
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].data.uint8[0] = _enabled ? 1 : 0;
  Log.noticeln(F("Disable module %d"), _id);
}

boolean DroneModule::isEnabled() {
  return _enabled;
}

void DroneModule::setup() {
  // subscribe to subs
  for (uint8_t i=0; i<_numSubs; i++) {
    _dlm->subscribe(&_subs[i].addr, this);
  }
}

void DroneModule::update() {

}

boolean DroneModule::readyToLoop() {
  if (!_enabled || _error > 0) return false;

  return (millis() > _lastLoop + _loopInterval);
}

void DroneModule::loop() {
  _lastLoop = millis();
}
