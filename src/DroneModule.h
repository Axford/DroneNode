/*
Base class for modules.
At creation:
* Passed a DroneModuleManager - stored and used to register itself
* Passed a DroneLinkManager - stored and used to wire up subs.

By convention, the module is registered to the channel matching its id.

things that would be useful for all modules
 - check if its registered
 - enabled/disabled
 - class name
 - name
 - error code or text
 - trigger a reset
*/

#ifndef DRONE_MODULE_H
#define DRONE_MODULE_H

#include "Arduino.h"
#include "DroneLinkMsg.h"
#include "ArduinoJson.h"
#include <ArduinoLog.h>
#include "DroneExecutionManager.h"
#include "strings.h"
#include <ESPAsyncWebServer.h>
#include <functional>

// defines for common params
#define DRONE_MODULE_PARAM_STATUS      1  // 0=disabled, 1=enabled, write a value of 2 or above to trigger reset
#define DRONE_MODULE_PARAM_NAME        2  // text
#define DRONE_MODULE_PARAM_ERROR       3  // text or error code, module defined implementation
#define DRONE_MODULE_PARAM_RESETCOUNT  4  // resetCount
#define DRONE_MODULE_PARAM_TYPE        5  // classname - text
#define DRONE_MODULE_PARAM_INTERVAL    6

// indices
#define DRONE_MODULE_PARAM_STATUS_E      0
#define DRONE_MODULE_PARAM_NAME_E        1
#define DRONE_MODULE_PARAM_ERROR_E       2
#define DRONE_MODULE_PARAM_RESETCOUNT_E  3
#define DRONE_MODULE_PARAM_TYPE_E        4
#define DRONE_MODULE_PARAM_INTERVAL_E    5

#define DRONE_MGMT_PARAM_ENTRIES        6

#define DRONE_CUSTOM_PARAM_START        8  // address at which custom params start


// other
#define DRONE_MODULE_RESET_INTERVAL  10000  // ms between reset attempts


struct DRONE_PARAM_ENTRY {
  uint8_t param;  // the param value as sent by dronelink
  const __FlashStringHelper *name; // pointer to PROGMEM string entry that describes this param
  uint8_t nameLen;
  uint8_t paramTypeLength;
  boolean publish;
  DRONE_LINK_PAYLOAD data;
};

// common structure for sub (input) params
// combining an address and a cache of the last value received
struct DRONE_PARAM_SUB {
  uint8_t addrParam; // param address of the addr value
  boolean received;  // set to true once first value received
  DRONE_LINK_ADDR addr;
  DRONE_PARAM_ENTRY param;
};

enum DRONE_MODULE_DISCOVERY_STATE {
  DRONE_MODULE_DISCOVERY_PENDING,
  DRONE_MODULE_DISCOVERY_MGMT,
  DRONE_MODULE_DISCOVERY_SUBS,
  DRONE_MODULE_DISCOVERY_SUB_ADDRS,
  DRONE_MODULE_DISCOVERY_PARAMS,
  DRONE_MODULE_DISCOVERY_COMPLETE
};

// forward declarations
class DroneLinkManager;
class DroneModuleManager;
class DroneExecutionManager;
//class DroneLinkMsg;


// DroneModule
class DroneModule {
protected:
  DroneLinkManager* _dlm;
  DroneModuleManager* _dmm;
  DroneExecutionManager* _dem;
  uint8_t _id;
  boolean _enabled;
  uint8_t _error;
  DroneLinkMsg _mgmtMsg;
  uint8_t _resetCount;
  unsigned long _lastReset;
  DRONE_PARAM_ENTRY *_mgmtParams;

  uint8_t _numParamEntries;
  DRONE_PARAM_ENTRY *_params;

  uint8_t _numSubs;
  DRONE_PARAM_SUB *_subs;
  boolean _updateNeeded;

  DRONE_MODULE_DISCOVERY_STATE _discoveryState;
  uint8_t _discoveryIndex;

  unsigned long _lastLoop;
  boolean _setupDone;

public:
  long hLMDuration;
  long loopDuration;

  DroneModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);
  ~DroneModule();

  uint8_t id();

  void setTypeName( const __FlashStringHelper *name);
  void setParamName( const __FlashStringHelper *name, DRONE_PARAM_ENTRY *param);
  char* getName();

  virtual void initSubs(uint8_t numSubs);
  virtual void initParams(uint8_t numParams);

  uint8_t getParamIdByName(const char* name);
  uint8_t getSubIdByName(const char* name);
  DRONE_PARAM_ENTRY* getParamEntryByName(const char* name);
  DRONE_PARAM_SUB* getSubByName(const char* name);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerConstructor(DEM_NAMESPACE* ns, DroneExecutionManager *dem);
  static void registerMgmtParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void reset();
  virtual void doReset();

  // notified prior to a restart or shutdown
  virtual void doShutdown();

  // in case modules want to display a progress indicator when an OTA is in progress
  virtual void onOTAProgress(float progress);

  virtual boolean isAlive();  // polled by watchdog, return true if everything ok

  //virtual void parsePins(JsonObject &obj, uint8_t *pins, uint8_t numPins);
  virtual void loadConfiguration(JsonObject &obj);

  boolean publishParamEntry(DRONE_PARAM_ENTRY *param);
  boolean publishMgmtParamEntries();
  virtual boolean publishParamEntries();

  boolean publishSubAddress(DRONE_PARAM_SUB *sub);
  virtual boolean publishSubs();

  virtual void onParamWrite(DRONE_PARAM_ENTRY *param);

  // write new value(s) to a param, and publish if it has changed
  void updateAndPublishParam(DRONE_PARAM_ENTRY *param, uint8_t *newPayload, uint8_t length);

  void handleParamMessage(DroneLinkMsg *msg, DRONE_PARAM_ENTRY *param);
  void handleSubAddrMessage(DroneLinkMsg *msg, DRONE_PARAM_SUB *sub);
  virtual boolean handleManagementMessage(DroneLinkMsg *msg);
  virtual void handleLinkMessage(DroneLinkMsg *msg);

  void setError(uint8_t error);

  virtual void enable();
  virtual void disable();
  virtual boolean isEnabled();

  virtual void setup();

  void updateIfNeeded();
  virtual void update();

  virtual boolean readyToLoop();
  virtual void loop();

  boolean doDiscovery();  // returns true when complete
  void restartDiscovery();

  void respondWithInfo(AsyncResponseStream *response);
};


#endif
