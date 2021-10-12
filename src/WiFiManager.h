#ifndef WIFI_KEEP_CONNECTED_
#define WIFI_KEEP_CONNECTED_

#include "Arduino.h"
#include "LinkedList.h"
#include <ArduinoJson.h>
#include <ArduinoLog.h>

#define WIFI_TIMEOUT_MS       3000
#define WIFI_RECOVER_TIME_MS  10000

struct WiFiNetworkCredentials {
  char *ssid;
  char *password;
};

class WiFiManager {
public:

  WiFiManager();

  void start();
  void loadConfiguration();

private:
  IvanLinkedList::LinkedList<WiFiNetworkCredentials*> _networks;

  uint8_t _activeNetwork;
  int _taskCore;
  int _taskPriority;
  TaskHandle_t _Task1;
  unsigned long _connectionCounter;
  boolean _APMode;

  static void keepWiFiAlive(void *pvParameters);
};


#endif
