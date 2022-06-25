/*

Steps to setup a new device:
 - Setup project for device: esp32doit-devkit-v1
 - Adjust WIFI settings in main.cpp
 - Alter platform.ini for appropriate programming port (e.g. COM4)
 - Program the esp32
 - Upload the template flash (SPIFFS) filesystem..
     - Open terminal inside platform.io
     - Run: pio run --target uploadfs
 - Open terminal to ESP32 port at 115200
 - Reboot the ESP32
 - Wait until IP address is shown (obtained via DHCP)
 - Connect in web-browser to /edit (e.g. 192.168.1.1/edit)
 - SPIFSS editor should be displayed
 - Open example hullConfig.json file and save as config.json (exact case)
 - Alter to suit project
 - Reboot ESP and watch terminal to ensure config is loaded correctly

*/

//#define CONFIG_ARDUINO_LOOP_STACK_SIZE 16384

// arduino core
#include <functional>
#include <Arduino.h>

// drone core
#include "DroneSystem.h"

//#define INC_WEB_SERVER

// web services

#include <AsyncTCP.h>
#include <ESPmDNS.h>

#ifdef INC_WEB_SERVER
#include <ESPAsyncWebServer.h>
#include "WebFSEditor.h"
#endif

//#include <AsyncJson.h>
//#include <ArduinoJson.h>

#ifdef INC_SPIFFS_EDITOR
#include <SPIFFSEditor.h>
#endif

//#include "WiFiKeepConnected.h"
//#include "WiFiManager.h"
#include "OTAManager.h"
//#include <ESP32Servo.h>
//#include <ArduinoLog.h>

// config
//#include "pinConfig.h"

DroneSystem ds;

#ifdef INC_WEB_SERVER
AsyncWebServer server(80);
WebFSEditor fsEditor(LITTLEFS, doLoop);
#endif

AsyncEventSource events("/events");
OTAManager OTAMgr(&events);


const char* QUERY_PARAM_APMODE = "APMode";
const char* QUERY_PARAM_SSID = "ssid";
const char* QUERY_PARAM_PASSWORD = "password";


#ifdef INC_WEB_SERVER
void setupWebServer() {
  Log.noticeln(F("[] Setup web server..."));
  events.onConnect([](AsyncEventSourceClient *client){
    client->send("hello!",NULL,millis(),1000);
  });
  server.addHandler(&events);

  // node info debug
  server.on("/nodeInfo", HTTP_GET, [](AsyncWebServerRequest *request){
    doLoop = false;
    dlm->serveNodeInfo(request);
    doLoop = true;
  });

  // channel info debug
  server.on("/channelInfo", HTTP_GET, [](AsyncWebServerRequest *request){
    doLoop = false;
    dlm->serveChannelInfo(request);
    doLoop = true;
  });

  // modules
  server.on("/modules", HTTP_GET, [](AsyncWebServerRequest *request){
    doLoop = false;
    dmm->serveModuleInfo(request);
    doLoop = true;
  });

  // DEM handlers
  server.on("/macros", HTTP_GET, [](AsyncWebServerRequest *request){
    doLoop = false;
    dem->serveMacroInfo(request);
    doLoop = true;
  });
  server.on("/execution", HTTP_GET, [](AsyncWebServerRequest *request){
    doLoop = false;
    dem->serveExecutionInfo(request);
    doLoop = true;
  });
  server.on("/commands", HTTP_GET, [](AsyncWebServerRequest *request){
    doLoop = false;
    dem->serveCommandInfo(request);
    doLoop = true;
  });

  /*
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404);
  });
  */

  fsEditor.httpuser = "admin";
  fsEditor.httppassword = "admin";
  fsEditor.xSPISemaphore = xSPISemaphore;
  fsEditor.configureWebServer(server);

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

  server.serveStatic("/", LITTLEFS, "/").setDefaultFile("index.htm");

  server.begin();
}
#endif


void handleOTAEVent(OTAManagerEvent event, float progress) {
  if (event == start) {
    // TODO
    //dmm->shutdown();
  } else if (event == progress) {
    Log.noticeln(F("OTA progress: %f"), progress*100);
    // TODO
    //dmm->onOTAProgress(progress);
  }
}


void handleDLMEvent( DroneLinkManagerEvent event, float progress) {
  if (event == DRONE_LINK_MANAGER_FIRMWARE_UPDATE_START) {
    //TODO
    //dmm->updateStarting();

  } else if (event == DRONE_LINK_MANAGER_FIRMWARE_UPDATE_END) {

  }
}

/*
TaskHandle_t _coreTask;

void coreTask( void * pvParameters ) {
  for(;;){
    if (!OTAMgr.isUpdating) {
      dmm->watchdog();

      yield();

      //Log.noticeln("[] lM");
      dmm->loopModules();

      yield();

      //Log.noticeln("[] pC");
      dlm->processChannels();

      yield();

      //Log.noticeln("[] e");
      dem->execute();
    } else {
      digitalWrite(PIN_LED, HIGH);
    }
    vTaskDelay(50);
  }
}
*/

void setup() {
  ds.setup();

  //MDNS.addService("http","tcp",80);

  /*
  #ifdef INC_WEB_SERVER
  setupWebServer();
  #endif
  */

  // TODO - integrate OTA
  /*
  OTAMgr.onEvent = handleOTAEVent;
  OTAMgr.init( dmm->hostname() );
  */
}


void loop() {
  ds.loop();
}
