#include "DroneModule.h"
#include "DroneLinkMsg.h"
#include "DroneLinkManager.h"
#include "DroneModuleManager.h"
#include "DroneExecutionManager.h"
#include "pinConfig.h"
#include "strings.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;


DroneModule::DroneModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
_dlm(dlm),
_dmm(dmm),
_dem(dem),
_id(id),
_fs(fs) {
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
  _lastLoop = 0;
  _discoveryState = DRONE_MODULE_DISCOVERY_PENDING;
  _discoveryIndex = 0;
  _setupDone = false;
  _updateNeeded = false;
  hLMDuration = 0;
  loopDuration = 0;

  // alloc for mgmt params
  _mgmtParams = (DRONE_PARAM_ENTRY*)malloc( sizeof(DRONE_PARAM_ENTRY) * DRONE_MGMT_PARAM_ENTRIES);

  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, DRONE_MODULE_PARAM_STATUS);
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].name = FPSTR(STRING_STATUS);
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].nameLen = sizeof(STRING_STATUS);
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].publish = true;
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].data.uint8[0] = _enabled ? 1 : 0;

  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, DRONE_MODULE_PARAM_NAME);
  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].name = FPSTR(STRING_NAME);
  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].nameLen = sizeof(STRING_NAME);
  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_CHAR, 1);
  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].publish = true;
  _mgmtParams[DRONE_MODULE_PARAM_NAME_E].data.c[0] = '?';

  _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, DRONE_MODULE_PARAM_ERROR);
  _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].name = FPSTR(STRING_ERROR);
  _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].nameLen = sizeof(STRING_ERROR);
  _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
  _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].publish = true;
  _mgmtParams[DRONE_MODULE_PARAM_ERROR_E].data.uint8[0] = _error;

  _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, DRONE_MODULE_PARAM_RESETCOUNT);
  _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].name = FPSTR(STRING_RESETCOUNT);
  _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].nameLen = sizeof(STRING_RESETCOUNT);
  _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
  _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].publish = true;
  _mgmtParams[DRONE_MODULE_PARAM_RESETCOUNT_E].data.uint8[0] = _resetCount;

  _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, DRONE_MODULE_PARAM_TYPE);
  _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].name = FPSTR(STRING_TYPE);
  _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].nameLen = sizeof(STRING_TYPE);
  _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].publish = true;
  _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, sizeof(STRING_DRONE));
  strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, STRING_DRONE, sizeof(STRING_DRONE));

  _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, DRONE_MODULE_PARAM_INTERVAL);
  _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].name = FPSTR(STRING_INTERVAL);
  _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].nameLen = sizeof(STRING_INTERVAL);
  _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].publish = true;
  _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
  _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 0;  // default no delay between loop calls

}


DroneModule::~DroneModule() { }

uint8_t DroneModule::id() {
  return _id;
}


void DroneModule::setTypeName( const __FlashStringHelper * name) {
  int len = strlen_P((PGM_P)name);
  _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, len);
  strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, (PGM_P)name, len);
}


void DroneModule::setParamName( const __FlashStringHelper * name, DRONE_PARAM_ENTRY *param) {
  param->name = name;
  param->nameLen = strlen_P((PGM_P)name);
}


char* DroneModule::getName() {
  // make sure it's null terminated
  uint8_t len = (_mgmtParams[DRONE_MODULE_PARAM_NAME_E].paramTypeLength & 0xF) + 1;
  if (len < DRONE_LINK_MSG_MAX_PAYLOAD)
    _mgmtParams[DRONE_MODULE_PARAM_NAME_E].data.c[len] = 0;
  return _mgmtParams[DRONE_MODULE_PARAM_NAME_E].data.c;
}


void DroneModule::initSubs(uint8_t numSubs) {
  _numSubs = numSubs;
  _subs = (DRONE_PARAM_SUB*)malloc( sizeof(DRONE_PARAM_SUB) * _numSubs);

  for (uint8_t i=0; i<_numSubs; i++) {
    _subs[i].received = false;
    _subs[i].enabled = true;
    _subs[i].addr.source = _dlm->node();
    _subs[i].addr.node = 0;
    _subs[i].addr.channel = 0;
    _subs[i].addr.paramPriority = 255;
    _subs[i].param.publish = false;
    _subs[i].param.nameLen = sizeof(STRING_BLANK);
    _subs[i].param.name = FPSTR(STRING_BLANK);
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


uint8_t DroneModule::getParamIdByName(const char* name) {
  DRONE_PARAM_ENTRY* p = getParamEntryByName(name);
  if (p != NULL) {
    return getDroneLinkMsgParam(p->paramPriority);
  }
  return 0;
}

uint8_t DroneModule::getSubIdByName(const char* name) {
  DRONE_PARAM_SUB* p = getSubByName(name);
  if (p != NULL) {
    return p->addrParam;
  }
  return 0;
}


DRONE_PARAM_ENTRY* DroneModule::getParamEntryByName(const char* name) {
  // check mgmt params first
  for (uint8_t i=0; i<DRONE_MGMT_PARAM_ENTRIES; i++) {
    if (strcmp_P(name, (PGM_P)_mgmtParams[i].name)==0) {
      return &_mgmtParams[i];
    }
  }

  for (uint8_t i=0; i<_numParamEntries; i++) {
    if (strcmp_P(name, (PGM_P)_params[i].name)==0) {
      return &_params[i];
    }
  }

  for (uint8_t i=0; i<_numSubs; i++) {
    if (strcmp_P(name, (PGM_P)_subs[i].param.name)==0) {
      return &_subs[i].param;
    }
  }

  return NULL;
}

DRONE_PARAM_SUB* DroneModule::getSubByName(const char* name) {
  for (uint8_t i=0; i<_numSubs; i++) {
    if (strcmp_P(name, (PGM_P)_subs[i].param.name)==0) {
      return &_subs[i];
    }
  }

  return NULL;
}



DEM_NAMESPACE* DroneModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(PSTR("Drone"),0,true);
}

void DroneModule::registerConstructor(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  // constructor
  DEMCommandHandler ch = std::bind(&DroneExecutionManager::mod_constructor, dem, _1, _2, _3, _4);
  dem->registerCommand(ns, STR_NEW, DRONE_LINK_MSG_TYPE_UINT8_T, ch);
}

void DroneModule::registerMgmtParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
    // writable mgmt params
    DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);

    dem->registerCommand(ns, STRING_STATUS, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
    dem->registerCommand(ns, STRING_NAME, DRONE_LINK_MSG_TYPE_CHAR, ph);
    dem->registerCommand(ns, STRING_INTERVAL, DRONE_LINK_MSG_TYPE_UINT32_T, ph);
}

void DroneModule::reset() {
  unsigned long loopTime = millis();
  if (_setupDone && loopTime > _lastReset + DRONE_MODULE_RESET_INTERVAL) {
    Log.warningln(F("[DM.r] Resetting %d"), _id);
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

// to be overridden
uint8_t DroneModule::getInterfaceType() { return DRONE_MESH_INTERFACE_TYPE_UDP; }
boolean DroneModule::getInterfaceState() { return false; }


boolean DroneModule::publishParamEntry(DRONE_PARAM_ENTRY *param) {
  _mgmtMsg._msg.paramPriority = param->paramPriority;
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
  _mgmtMsg._msg.paramPriority = sub->addrParam;
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
  if (getDroneLinkMsgParam(param->paramPriority) == DRONE_MODULE_PARAM_STATUS) {
    switch(param->data.uint8[0]) {
      case 0: disable();
        break;
      case 1: enable();
        break;
    }
  }
}


void DroneModule::updateAndPublishParam(DRONE_PARAM_ENTRY *param, uint8_t *newPayload, uint8_t length) {
  if (memcmp(param->data.c, newPayload, length) != 0) {
    memcpy(param->data.c, newPayload, length);
    // publish
    if (param->publish) publishParamEntry(param);
  }
}


void DroneModule::handleParamMessage(DroneLinkMsg *msg, DRONE_PARAM_ENTRY *param) {
  //Log.noticeln("[DM.hPM]");

  if (msg->type() == ((param->paramTypeLength >> 4) & 0x07)) {
    // write param (the writable bit must be cleared to set a value, to avoid published values writing to themselves!)
    if ( ((param->paramTypeLength & DRONE_LINK_MSG_WRITABLE) > 0) &&
         !msg->writable()
       ) {

      uint8_t len = msg->length();
      // compare to see if anything has changed... which may including receiving our own messages after a query!!
      if (memcmp(param->data.c, msg->_msg.payload.c, len) != 0) {
        //Log.noticeln("Write to param: ");
        //msg->print();
        //Log.noticeln("[DM.hPM] memcpy");
        memcpy(param->data.c, msg->_msg.payload.c, len);

        // update param paramTypeLength
        param->paramTypeLength = (param->paramTypeLength & 0b11110000) | ((len-1) & 0xF);

        //Log.noticeln("[DM.hPM] onParamWrite");
        // trigger callback
        onParamWrite(param);
        //Log.noticeln("[DM.hPM] update");
        //update();
        _updateNeeded = true;
      }


    }

  } else if (msg->type() == DRONE_LINK_MSG_TYPE_QUERY) {
    // query -> publish
    publishParamEntry(param);

  } else if (msg->type() == DRONE_LINK_MSG_TYPE_NAMEQUERY) {
    // query name
    // set return priority to low for names
    _mgmtMsg._msg.paramPriority = getDroneLinkMsgParam(param->paramPriority);
    _mgmtMsg.type(DRONE_LINK_MSG_TYPE_NAME);
    _mgmtMsg.length(param->nameLen);
    memcpy(_mgmtMsg._msg.payload.c, param->name, param->nameLen);
    _dlm->publish(_mgmtMsg);
  } else {
    //Log.warningln("[DM.hPM] Attempt to write mismatching type %u > %u", msg->type(), ty );
  }
  //Log.noticeln("[DM.hPM] end");
}

void DroneModule::handleSubAddrMessage(DroneLinkMsg *msg, DRONE_PARAM_SUB *sub) {
  if (msg->type() == DRONE_LINK_MSG_TYPE_ADDR &&
      !msg->writable()) {
    // write new address
    uint8_t len = msg->length();  // should always be sizeof(DRONE_LINK_ADDR) //msg->length();
    // compare to see if anything has changed... which may including receiving our own messages after a query!!
    if (memcmp((uint8_t*)&sub->addr, msg->_msg.payload.c, len) != 0) {
      Log.noticeln("[DM.hSAM] Write to address: ");
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
    // set priority to low for names
    _mgmtMsg._msg.paramPriority = sub->addrParam;  // sub addr always has low upper bits = low priority
    _mgmtMsg.type(DRONE_LINK_MSG_TYPE_NAME);
    _mgmtMsg.length(sub->param.nameLen);
    memcpy(_mgmtMsg._msg.payload.c, sub->param.name, sub->param.nameLen);
    _dlm->publish(_mgmtMsg);
  }
}


boolean DroneModule::handleManagementMessage(DroneLinkMsg *msg) {
  //Log.noticeln("[DM.hMM]");
  if (msg->param() < DRONE_CUSTOM_PARAM_START) {
    for (uint8_t i=0; i<DRONE_MGMT_PARAM_ENTRIES; i++) {
      if (msg->param() == getDroneLinkMsgParam(_mgmtParams[i].paramPriority)) {
        handleParamMessage(msg, &_mgmtParams[i]);
      }
    }
    //Log.noticeln("[DM.hMM] end");
    return true;
  }
  return false;
}


void DroneModule::handleLinkMessage(DroneLinkMsg *msg) {
  unsigned long start = millis();
  //Log.noticeln("[DM.hLM]");
  //msg->print();

  // handle subs
  if (_enabled || !_setupDone) {
    for (uint8_t i=0; i<_numSubs; i++) {
      if (_subs[i].enabled && msg->sameAddress(&_subs[i].addr)) {

        // check the type matches what's expected
        if (msg->type() == ((_subs[i].param.paramTypeLength >> 4) & 0x7)) {
          // handle the subscription updating
          memcpy(_subs[i].param.data.c, msg->_msg.payload.c, msg->length());

          _subs[i].received = true;

          // match the param length
          _subs[i].param.paramTypeLength = (_subs[i].param.paramTypeLength & 0xF0) | (msg->length()-1);

          // publish the change to this param
          if (_subs[i].param.publish) publishParamEntry(&_subs[i].param);

          // trigger an update
          //update();
          _updateNeeded = true;
        }
      }
    }
  }

  if (msg->node() != _dlm->node() || msg->channel() != _id) return;

  if (!handleManagementMessage(msg)) {

    if (_enabled || !_setupDone) {
      // handle output params
      for (uint8_t i=0; i<_numParamEntries; i++) {
        if (msg->param() == getDroneLinkMsgParam(_params[i].paramPriority)) {
          handleParamMessage(msg, &_params[i]);
        }
      }

      // handle subs
      for (uint8_t i=0; i<_numSubs; i++) {
        if (msg->param() == getDroneLinkMsgParam(_subs[i].param.paramPriority)) {
          // handles writes to the internal value
          handleParamMessage(msg, &_subs[i].param);
        } else if (msg->param() == _subs[i].addrParam) {
          // handle writes to the address
          handleSubAddrMessage(msg, &_subs[i]);
        }
      }
    }
  }

  //Log.noticeln("[DM.hLM] end");
  long duration = millis() - start;
  if (duration > hLMDuration) hLMDuration = duration;
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
  Log.noticeln(F("[DM.en] enable %d"), _id);
}

void DroneModule::disable() {
  if (!_enabled) return;
  _enabled = false;
  _mgmtParams[DRONE_MODULE_PARAM_STATUS_E].data.uint8[0] = _enabled ? 1 : 0;
  Log.noticeln(F("[DM.dis] disable %d"), _id);
}

boolean DroneModule::isEnabled() {
  return _enabled;
}

void DroneModule::setup() {
  if (_setupDone) return;
  // subscribe to subs
  for (uint8_t i=0; i<_numSubs; i++) {
    _dlm->subscribe(&_subs[i].addr, this);
  }
  _setupDone = true;
}

void DroneModule::updateIfNeeded() {
  if (_updateNeeded && _setupDone && _enabled && _error == 0) {
    update();
    _updateNeeded = false;
  }
}

void DroneModule::update() {

}

boolean DroneModule::readyToLoop() {
  if (!_setupDone || !_enabled || _error > 0) return false;

  return (millis() > _lastLoop + _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0]);
}

void DroneModule::loop() {
  _lastLoop = millis();
}


boolean DroneModule::doDiscovery() {
  //Log.noticeln("Discover module: %u", _id);
  switch (_discoveryState) {
    case DRONE_MODULE_DISCOVERY_PENDING:
      _discoveryState = DRONE_MODULE_DISCOVERY_MGMT;
      _discoveryIndex = 0;
      // flow straight into mgmt discovery

    case DRONE_MODULE_DISCOVERY_MGMT:
      if (_discoveryIndex < DRONE_MGMT_PARAM_ENTRIES) {
        if (_mgmtParams[_discoveryIndex].publish) {
          //Log.noticeln("Discover mgmt: %u", _discoveryIndex);
          publishParamEntry(&_mgmtParams[_discoveryIndex]);
        }
      }
      _discoveryIndex++;
      if (_discoveryIndex >= DRONE_MGMT_PARAM_ENTRIES ) {
        _discoveryState = DRONE_MODULE_DISCOVERY_SUBS;
        _discoveryIndex = 0;
      }
      break;

    case DRONE_MODULE_DISCOVERY_SUBS:
      if (!_enabled) {
        _discoveryState = DRONE_MODULE_DISCOVERY_COMPLETE;
        return true;
      }
      if (_discoveryIndex < _numSubs) {
        if (_subs[_discoveryIndex].param.publish) {
          //Log.noticeln("Discover sub: %u . %u", _id, _subs[_discoveryIndex].param.param);
          publishParamEntry(&_subs[_discoveryIndex].param);
          //publishSubAddress(&_subs[_discoveryIndex]);
        }
      }
      _discoveryIndex++;
      if (_discoveryIndex >= _numSubs ) {
        _discoveryState = DRONE_MODULE_DISCOVERY_SUB_ADDRS;
        _discoveryIndex = 0;
      }
      break;

    case DRONE_MODULE_DISCOVERY_SUB_ADDRS:
      if (!_enabled) {
        _discoveryState = DRONE_MODULE_DISCOVERY_COMPLETE;
        return true;
      }
      if (_discoveryIndex < _numSubs) {
        if (_subs[_discoveryIndex].param.publish) {
          //Log.noticeln("Discover sub addr: %u . %u", _id, _subs[_discoveryIndex].addrParam);
          //publishParamEntry(&_subs[_discoveryIndex].param);
          publishSubAddress(&_subs[_discoveryIndex]);
        }
      }
      _discoveryIndex++;
      if (_discoveryIndex >= _numSubs ) {
        _discoveryState = DRONE_MODULE_DISCOVERY_PARAMS;
        _discoveryIndex = 0;
      }
      break;

    case DRONE_MODULE_DISCOVERY_PARAMS:
      if (!_enabled) {
        _discoveryState = DRONE_MODULE_DISCOVERY_COMPLETE;
        return true;
      }
      if (_discoveryIndex < _numParamEntries) {
        if (_params[_discoveryIndex].publish) {
          //Log.noticeln("Discover param: %u", _discoveryIndex);
          publishParamEntry(&_params[_discoveryIndex]);
        }
      }
      _discoveryIndex++;
      if (_discoveryIndex >= _numParamEntries ) {
        _discoveryState = DRONE_MODULE_DISCOVERY_COMPLETE;
        _discoveryIndex = 0;
        return true;
      }
      break;

    case DRONE_MODULE_DISCOVERY_COMPLETE:
      return true;
      break;
  }

  return false;
}

void DroneModule::restartDiscovery() {
  _discoveryState = DRONE_MODULE_DISCOVERY_PENDING;
  _discoveryIndex = 0;
}


void DroneModule::respondWithInfo(AsyncResponseStream *response) {
  DRONE_PARAM_ENTRY *p;

  response->print(F("  Stats:\n"));
  response->print("    HLM Dur: "); response->print(hLMDuration);
  response->print("\n    Loop Dur: ");  response->print(loopDuration);
  response->print("\n");

  response->print(F("\n  Mgmt Params:\n"));
  for (uint8_t i=0; i<DRONE_MGMT_PARAM_ENTRIES; i++) {
    p = &_mgmtParams[i];

    response->printf("    %u: %s ",getDroneLinkMsgParam(p->paramPriority),(PGM_P)p->name);
    if (p->publish) response->print('*');
    DroneLinkMsg::printPayload(&p->data, p->paramTypeLength, response);
  }

  response->print(F("\n  Params:\n"));
  for (uint8_t i=0; i<_numParamEntries; i++) {
    p = &_params[i];
    response->printf("    %u: %s ",getDroneLinkMsgParam(p->paramPriority),(PGM_P)(p->name));
    if (p->publish) response->print('*');
    DroneLinkMsg::printPayload(&p->data, p->paramTypeLength, response);
  }

  response->print(F("\n  Subs:\n"));
  DRONE_PARAM_SUB *s;
  for (uint8_t i=0; i<_numSubs; i++) {
    s = &_subs[i];
    response->printf("    %u: $%s ", s->addrParam, (PGM_P)s->param.name);
    if (s->param.publish) response->print('*');
    response->print('[');
    DroneLinkMsg::printAddress(&s->addr, response);
    response->print("]\n");

    response->printf("    %u: %s ",getDroneLinkMsgParam(s->param.paramPriority), (PGM_P)s->param.name);
    if (s->param.publish) response->print('*');
    DroneLinkMsg::printPayload(&s->param.data, s->param.paramTypeLength, response);
  }
  response->print(F("\n"));
}


uint8_t DroneModule::diagnosticDisplays() {
  return 0;
}

void DroneModule::drawDiagnosticDisplay(SSD1306Wire *display, uint8_t page) {

}
