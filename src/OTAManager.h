#ifndef OTA_MANAGER_
#define OTA_MANAGER_

#include "Arduino.h"
#include <ESPAsyncWebServer.h>
#include "WiFi.h"
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

class OTAManager {
  public:
    boolean isUpdating;
    OTAManager(AsyncEventSource * events);

    void init(String hostname);
    void loop();
  private:
    AsyncEventSource *_events;
};

#endif
