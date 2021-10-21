#ifndef WIFI_KEEP_CONNECTED_
#define WIFI_KEEP_CONNECTED_

#include "Arduino.h"
#include "LinkedList.h"
#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include "DroneModuleManager.h"
#include "FS.h"

#define WIFI_TIMEOUT_MS       10000
#define WIFI_RECOVER_TIME_MS  10000

struct WiFiNetworkCredentials {
  String ssid;
  String password;
};

class WiFiManager {
public:

  WiFiManager();

  void start();
  void loadConfiguration(fs::FS &fs);

protected:
  IvanLinkedList::LinkedList<WiFiNetworkCredentials> _networks;

  boolean _scanActive;
  boolean _attemptingConnection;
  uint8_t _activeNetwork;
  int _taskCore;
  int _taskPriority;
  TaskHandle_t _Task1;
  unsigned long _connectionCounter;
  boolean _APMode;

  static void keepWiFiAlive(void *pvParameters);
};


#endif
