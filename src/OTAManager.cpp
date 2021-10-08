#include "OTAManager.h"
#include "Arduino.h"
#include "SPIFFS.h"
#include <esp_task_wdt.h>

OTAManager::OTAManager(AsyncEventSource * events) {
  _events = events;
  isUpdating = false;
}


void OTAManager::init(String hostname) {

  Serial.println(F("[OTA] Configuring OTA"));

  //Send OTA events to the browser
  ArduinoOTA.onStart([&]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    SPIFFS.end();
    Serial.println("[OTA] Start updating " + type);
    isUpdating = true;

    // backoff watchdog
    esp_task_wdt_init(60,0);

    _events->send("Update Start", "ota");
    if (onEvent) { onEvent(start, 0); }
  });
  ArduinoOTA.onEnd([&]() {
    Serial.println(F("[OTA] End"));
    _events->send("Update End", "ota");
    esp_task_wdt_init(5,0);
    isUpdating = false;
    if (onEvent) { onEvent(end, 100); }
  });

  ArduinoOTA.onProgress([&](unsigned int prog, unsigned int total) {
    char p[32];
    //float progress = (float) prog / total;

    if (onEvent) { onEvent(progress, (float) prog / total); }

    sprintf(p, "Progress: %u%%\n", (prog/(total/100)));
    _events->send(p, "ota");
    //Serial.println(p);
  });
  ArduinoOTA.onError([&](ota_error_t error) {
    if(error == OTA_AUTH_ERROR) _events->send("Auth Failed", "ota");
    else if(error == OTA_BEGIN_ERROR) _events->send("Begin Failed", "ota");
    else if(error == OTA_CONNECT_ERROR) _events->send("Connect Failed", "ota");
    else if(error == OTA_RECEIVE_ERROR) _events->send("Recieve Failed", "ota");
    else if(error == OTA_END_ERROR) _events->send("End Failed", "ota");
  });

  ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname(const_cast<char*>(hostname.c_str()));
  ArduinoOTA.begin();
}

void OTAManager::loop() {
  ArduinoOTA.handle();
}
