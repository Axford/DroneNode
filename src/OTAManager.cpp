#include "OTAManager.h"
#include "Arduino.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "LITTLEFS.h"
#include <ArduinoLog.h>
#include <esp_task_wdt.h>

OTAManager::OTAManager(AsyncEventSource * events) {
  _events = events;
  isUpdating = false;
  //_firmwareUrl = "http://192.168.0.23/firmware";
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

    LITTLEFS.end();
    Serial.println("[OTA] Start updating " + type);
    isUpdating = true;

    // backoff watchdog
    esp_task_wdt_init(60,0);

    // clear interrupts
    cli();

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

/*
void OTAManager::execOTA()
{
    int contentLength = 0;
    bool isValidContentType = false;
    isUpdating = true;

    HTTPClient http;
    //http.setConnectTimeout( 1000 );

    Log.noticeln("Connecting to: %s\r\n", _firmwareUrl.c_str() );
    http.begin( _firmwareUrl );

    const char* get_headers[] = { "Content-Length", "Content-type" };
    http.collectHeaders( get_headers, 2 );

    int httpCode = http.GET();

    if( httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY ) {
        contentLength = http.header( "Content-Length" ).toInt();
        String contentType = http.header( "Content-type" );
        if( contentType == "application/octet-stream" ) {
            isValidContentType = true;

        }
    } else {
        // Connect to webserver failed
        // May be try?
        // Probably a choppy network?
        Log.noticeln( "Connection to %s failed. Please check your setup", _firmwareUrl );
        // retry??
        // execOTA();
    }

       // Check what is the contentLength and if content type is `application/octet-stream`
    Log.noticeln("contentLength : %i, isValidContentType : %s", contentLength, String(isValidContentType));

    // check contentLength and content type
    if( contentLength && isValidContentType ) {
        WiFiClient& client = http.getStream();

        // Check if there is enough to OTA Update
        bool canBegin = Update.begin(contentLength);

        // If yes, begin
        if( canBegin ) {
            Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quiet for a while.. Patience!");
            // No activity would appear on the Serial monitor
            // So be patient. This may take 2 - 5mins to complete
            size_t written = Update.writeStream(client);

            if (written == contentLength)
            {
                Serial.println("Written : " + String(written) + " successfully");
            }
            else
            {
                Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?");
                // retry??
                // execOTA();
            }

            if (Update.end())
            {
                Serial.println("OTA done!");
                if (Update.isFinished())
                {
                    Serial.println("Update successfully completed. Rebooting.");
                    http.end();
                    ESP.restart();
                }
                else
                {
                    Serial.println("Update not finished? Something went wrong!");
                }
            }
            else
            {
                Serial.println("Error Occurred. Error #: " + String(Update.getError()));
            }
        }
        else
        {
            // not enough space to begin OTA
            // Understand the partitions and
            // space availability
            Serial.println("Not enough space to begin OTA");
            http.end();
        }
    }
    else
    {
        Log.errorln("There was no content in the response");
        http.end();
    }
    isUpdating = false;
}
*/
