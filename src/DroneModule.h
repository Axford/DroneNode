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

// defines for common params
#define DRONE_MODULE_PARAM_STATUS      1  // 0=disabled, 1=enabled, write a value of 2 or above to trigger reset
#define DRONE_MODULE_PARAM_NAME        2  // text
#define DRONE_MODULE_PARAM_ERROR       3  // text or error code, module defined implementation
#define DRONE_MODULE_PARAM_RESETCOUNT  4  // resetCount
#define DRONE_MODULE_PARAM_TYPE        5  // classname - text

// indices
#define DRONE_MODULE_PARAM_STATUS_E      0
#define DRONE_MODULE_PARAM_NAME_E        1
#define DRONE_MODULE_PARAM_ERROR_E       2
#define DRONE_MODULE_PARAM_RESETCOUNT_E  3
#define DRONE_MODULE_PARAM_TYPE_E        4

#define DRONE_MGMT_PARAM_ENTRIES        5

#define DRONE_CUSTOM_PARAM_START        8  // address at which custom params start


// strings
static const char DRONE_STR_D[] PROGMEM = " %d ";
static const char DRONE_STR_F[] PROGMEM = " %F ";
static const char DRONE_STR_S[] PROGMEM = " %s ";
static const char DRONE_STR_X[] PROGMEM = " %X ";

static const char DRONE_STR_BLANK[] PROGMEM = "";

static const char DRONE_STR_ACCEL[] PROGMEM = "accel";
static const char DRONE_STR_ADDR[] PROGMEM = "addr";
static const char DRONE_STR_ALTITUDE[] PROGMEM = "altitude";
static const char DRONE_STR_BAUD[] PROGMEM = "baud";
static const char DRONE_STR_BUILD[] PROGMEM = "build";
static const char DRONE_STR_BUS[] PROGMEM = "bus";
static const char DRONE_STR_BUSV[] PROGMEM = "busV";
static const char DRONE_STR_BUTTON[] PROGMEM = "button";
static const char DRONE_STR_CALIB_X[] PROGMEM = "calibX";
static const char DRONE_STR_CALIB_Y[] PROGMEM = "calibY";
static const char DRONE_STR_CENTRE[] PROGMEM = "centre";
static const char DRONE_STR_CHOKED[] PROGMEM = "choked";
static const char DRONE_STR_CURRENT[] PROGMEM = "current";
static const char DRONE_STR_DEADBAND[] PROGMEM = "deadband";
static const char DRONE_STR_DECLINATION[] PROGMEM = "declination";
static const char DRONE_STR_DISTANCE[] PROGMEM = "distance";
static const char DRONE_STR_DRONE[] PROGMEM = "Drone";
static const char DRONE_STR_ERROR[] PROGMEM = "error";
static const char DRONE_STR_ENABLE[] PROGMEM = "enable";
static const char DRONE_STR_GYRO[] PROGMEM = "gyro";
static const char DRONE_STR_HDOP[] PROGMEM = "HDOP";
static const char DRONE_STR_HEADING[] PROGMEM = "heading";
static const char DRONE_STR_HEAP[] PROGMEM = "heap";
static const char DRONE_STR_HOSTNAME[] PROGMEM = "hostname";
static const char DRONE_STR_HUMIDITY[] PROGMEM = "humidity";
static const char DRONE_STR_INPUT[] PROGMEM = "input";
static const char DRONE_STR_INTERVAL[] PROGMEM = "interval";
static const char DRONE_STR_INVERT[] PROGMEM = "invert";
static const char DRONE_STR_IP[] PROGMEM = "IP";
static const char DRONE_STR_LEFT[] PROGMEM = "left";
static const char DRONE_STR_LOADV[] PROGMEM = "loadV";
static const char DRONE_STR_LOCATION[] PROGMEM = "location";
static const char DRONE_STR_LOOPTO[] PROGMEM = "loopTo";
static const char DRONE_STR_NAME[] PROGMEM = "name";
static const char DRONE_STR_PID[] PROGMEM = "PID";
static const char DRONE_STR_PINS[] PROGMEM = "pins";
static const char DRONE_STR_PORT[] PROGMEM = "port";
static const char DRONE_STR_POSITION[] PROGMEM = "position";
static const char DRONE_STR_POWER[] PROGMEM = "power";
static const char DRONE_STR_PRESSURE[] PROGMEM = "pressure";
static const char DRONE_STR_PUBLISH[] PROGMEM = "publish";
static const char DRONE_STR_PUBLISHRATE[] PROGMEM = "publishRate";
static const char DRONE_STR_PWM_CHANNEL[] PROGMEM = "PWMChannel";
static const char DRONE_STR_RESET[] PROGMEM = "reset";
static const char DRONE_STR_RESETCOUNT[] PROGMEM = "resetCount";
static const char DRONE_STR_RIGHT[] PROGMEM = "right";
static const char DRONE_STR_RSSI[] PROGMEM = "RSSI";
static const char DRONE_STR_SAMPLEINTERVAL[] PROGMEM = "sampleInterval";
static const char DRONE_STR_SATELLITES[] PROGMEM = "satellites";
static const char DRONE_STR_SHUNTV[] PROGMEM = "shuntV";
static const char DRONE_STR_SPEED[] PROGMEM = "speed";
static const char DRONE_STR_STATUS[] PROGMEM = "status";
static const char DRONE_STR_SUBSCRIBETO[] PROGMEM = "subs";
static const char DRONE_STR_SUB1[] PROGMEM = "sub1";
static const char DRONE_STR_SUB2[] PROGMEM = "sub2";
static const char DRONE_STR_SUB3[] PROGMEM = "sub3";
static const char DRONE_STR_SUB4[] PROGMEM = "sub4";
static const char DRONE_STR_TARGET[] PROGMEM = "target";
static const char DRONE_STR_TARGET_LOCATION[] PROGMEM = "targetLocation";
static const char DRONE_STR_TEMPERATURE[] PROGMEM = "temperature";
static const char DRONE_STR_TURN_RATE[] PROGMEM = "turnRate";
static const char DRONE_STR_TYPE[] PROGMEM = "type";
static const char DRONE_STR_UPTIME[] PROGMEM = "uptime";
static const char DRONE_STR_VECTOR[] PROGMEM = "vector";
static const char DRONE_STR_WAYPOINTS[] PROGMEM = "waypoints";
static const char DRONE_STR_WAYPOINT[] PROGMEM = "waypoint";
static const char DRONE_STR_XAXIS[] PROGMEM = "xAxis";
static const char DRONE_STR_YAXIS[] PROGMEM = "yAxis";
static const char DRONE_STR_ZAXIS[] PROGMEM = "zAxis";

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

// forward declarations
class DroneLinkManager;
class DroneModuleManager;
//class DroneLinkMsg;

// DroneModule
class DroneModule {
protected:
  DroneLinkManager* _dlm;
  DroneModuleManager* _dmm;
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

  unsigned long _loopInterval;
  unsigned long _lastLoop;

public:
  DroneModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm);
  ~DroneModule();

  void setTypeName( const __FlashStringHelper *name);
  void setParamName( const __FlashStringHelper *name, DRONE_PARAM_ENTRY *param);

  virtual void initSubs(uint8_t numSubs);
  virtual void initParams(uint8_t numParams);

  void reset();
  virtual void doReset();

  // notified prior to a restart or shutdown
  virtual void doShutdown();

  // in case modules want to display a progress indicator when an OTA is in progress
  virtual void onOTAProgress(float progress);

  virtual boolean isAlive();  // polled by watchdog, return true if everything ok

  virtual void parsePins(JsonObject &obj, uint8_t *pins, uint8_t numPins);
  virtual void loadConfiguration(JsonObject &obj);

  boolean publishParamEntry(DRONE_PARAM_ENTRY *param);
  boolean publishMgmtParamEntries();
  virtual boolean publishParamEntries();

  boolean publishSubAddress(DRONE_PARAM_SUB *sub);
  virtual boolean publishSubs();

  virtual void onParamWrite(DRONE_PARAM_ENTRY *param);

  void handleParamMessage(DroneLinkMsg *msg, DRONE_PARAM_ENTRY *param);
  void handleSubAddrMessage(DroneLinkMsg *msg, DRONE_PARAM_SUB *sub);
  virtual boolean handleManagementMessage(DroneLinkMsg *msg);
  virtual void handleLinkMessage(DroneLinkMsg *msg);

  void setError(uint8_t error);

  virtual void enable();
  virtual void disable();
  virtual boolean isEnabled();

  virtual void setup();

  virtual void update();

  virtual boolean readyToLoop();
  virtual void loop();
};


#endif
