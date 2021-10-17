/*
Manages all active modules.  Deals with the management channel (e.g. dynamic sub changes, activate/deactivate modules)
*/

#ifndef DRONE_MODULE_MANAGER_H
#define DRONE_MODULE_MANAGER_H

#include "Arduino.h"
#include "LinkedList.h"
#include <ESPAsyncWebServer.h>
//#include <ArduinoJson.h>

#define DRONE_MODULE_MANAGER_WATCHDOG_INTERVAL 10000
#define DRONE_MODULE_MANAGER_DISCOVERY_INTERVAL 250

// forward declarations
class DroneModule;
class DroneLinkManager;


// DroneModuleManager
class DroneModuleManager {
protected:
  uint8_t _node;
  String _hostname;
  String _buildTimestamp;
  boolean _doDiscovery;
  unsigned long _lastWatchdogCheck;
  unsigned long _lastDiscovery;
  uint8_t _lastDiscoveryIndex;
  IvanLinkedList::LinkedList<DroneModule*> _modules;
  DroneLinkManager* _dlm;

public:
  DroneModuleManager(DroneLinkManager* dlm);
  void registerModule(DroneModule *m);

  DroneModule* getModuleById(uint8_t id);
  DroneModule* getModuleByName(char * name);

  void node(uint8_t id);
  uint8_t node();
  void hostname(const char * name);
  String hostname();
  String buildTimestamp();
  boolean discovery(); // get discovery state
  void discovery(boolean v); // set discovery state

  //void loadConfiguration();
  //void loadModulesFromJSON(const JsonArray &array);

  uint8_t moduleCount();

  void onOTAProgress(float progress);

  void shutdown();
  void restart();  // will shutdown all modules, then restart

  void setupModules();
  void loopModules();

  void watchdog();

  void serveModuleInfo(AsyncWebServerRequest *request);

};


#endif
