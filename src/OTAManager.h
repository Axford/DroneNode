#ifndef OTA_MANAGER_
#define OTA_MANAGER_

#include "Arduino.h"
#include <ESPAsyncWebServer.h>
#include "WiFi.h"
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

enum OTAManagerEvent { start, end, progress };

typedef void (*OTAManagerCallback) (const OTAManagerEvent event, const float progress);

class OTAManager {
  public:
    OTAManagerCallback onEvent;

    boolean isUpdating;
    OTAManager(AsyncEventSource * events);

    void init(String hostname);
    void loop();

    //void execOTA();
  private:
    AsyncEventSource *_events;
    //String _firmwareUrl;
};

#endif
