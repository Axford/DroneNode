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

// arduino core
#include <functional>
#include <Arduino.h>

// drone core
#include "DroneLinkMsg.h"
#include "DroneLinkChannel.h"
#include "DroneLinkManager.h"
#include "DroneModule.h"
#include "DroneModuleManager.h"
#include "DroneWire.h"
#include "DroneExecutionManager.h"

#define INC_WEB_SERVER
#define INC_SPIFFS_EDITOR

// other libraries
#include "SPIFFS.h"
#include <AsyncTCP.h>
#include <ESPmDNS.h>

#ifdef INC_WEB_SERVER
#include <ESPAsyncWebServer.h>
#endif

//#include <AsyncJson.h>
#include <ArduinoJson.h>

#ifdef INC_SPIFFS_EDITOR
#include <SPIFFSEditor.h>
#endif

//#include "WiFiKeepConnected.h"
#include "WiFiManager.h"
#include "OTAManager.h"
#include <ESP32Servo.h>
#include <ArduinoLog.h>

// config
#include "pinConfig.h"

WiFiManager wifiManager;

#ifdef INC_WEB_SERVER
AsyncWebServer server(80);
#endif

AsyncEventSource events("/events");
OTAManager OTAMgr(&events);

DroneLinkManager dlm;
DroneModuleManager dmm(&dlm);
DroneExecutionManager dem(&dmm, &dlm);


// HTML web page to handle 3 input fields (input1, input2, input3)
/*
const char wifi_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  </head><body>
    <h1>DroneModule Wifi Settings</h1>
  <form action="/savewifi">
    APMode: <input type="checkbox" name="APMode"><br/>
    SSID: <input typ e="text" name="ssid"><br/>
    Password: <input type="text" name="password"><br/>
    <input type="submit" value="Save">
  </form>
</body></html>
)rawliteral";
*/

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

  // setup wifi settings form
  /*
  server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", wifi_html);
  });
  */


  // Save new wifi settings <ESP_IP>/savewifi?xxx
  /*
  server.on("/savewifi", HTTP_GET, [] (AsyncWebServerRequest *request) {

    if (request->hasParam(QUERY_PARAM_APMODE)) {
      APMode = (request->getParam(QUERY_PARAM_APMODE)->value() == "on");
    } else {
      APMode = false;
    }

    if (request->hasParam(QUERY_PARAM_SSID)) {
      ssid = request->getParam(QUERY_PARAM_SSID)->value();
    }

    if (request->hasParam(QUERY_PARAM_PASSWORD)) {
      password = request->getParam(QUERY_PARAM_PASSWORD)->value();
    }

    // build into new json file and save to SPIFFS
    File file = SPIFFS.open(F("/wifi.json"), FILE_WRITE);
    file.println("{");
      file.print(F("\"APMode\":"));
      file.print(APMode ? "true" : "false");
      file.print(",\n");

      file.print(F("\"ssid\":\""));
      file.print(ssid);
      file.print("\",\n");

      file.print(F("\"password\":\""));
      file.print(password);
      file.print("\"\n");
    file.println("}");
    file.close();

    request->send(200, "text/html", F("Settings saved - rebooting"));
    ESP.restart();
  });
  */


  // node info debug
  server.on("/nodeInfo", HTTP_GET, [](AsyncWebServerRequest *request){
    dlm.serveNodeInfo(request);
  });

  // channel info debug
  server.on("/channelInfo", HTTP_GET, [](AsyncWebServerRequest *request){
    dlm.serveChannelInfo(request);
  });

  // dem macros
  server.on("/macros", HTTP_GET, [](AsyncWebServerRequest *request){
    dem.serveMacroInfo(request);
  });

  // dem commands
  server.on("/commands", HTTP_GET, [](AsyncWebServerRequest *request){
    dem.serveCommandInfo(request);
  });


  #ifdef INC_SPIFFS_EDITOR
  server.addHandler(new SPIFFSEditor(SPIFFS));
  #endif

  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404);
  });
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");

  server.begin();
}
#endif


void handleOTAEVent(OTAManagerEvent event, float progress) {
  if (event == start) {
    dmm.shutdown();
  } else if (event == progress) {
    Serial.print(F("OTA progress: "));  Serial.println(progress*100);
    dmm.onOTAProgress(progress);
  }
}


void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  Serial.begin(115200);
  while(!Serial) {  }

  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  delay(2500); // to allow serial to reconnect after programming

  // direct log output to file in SPIFFS
  Log.noticeln(F("[] Starting SPIFFS"));
  if (!SPIFFS.begin(true)) { // formatonfail=true
    Log.errorln(F("[] Unable to begin SPIFFS"));
  }
  Log.noticeln(F("  totalBytes %d"), SPIFFS.totalBytes());
  Log.noticeln(F("  usedBytes %d"), SPIFFS.usedBytes());

  File logFile = SPIFFS.open("/startup.log", FILE_WRITE);

  // switch to logging to startup.log file on spiffs
  //Log.begin(LOG_LEVEL_VERBOSE, &logFile);
  Log.noticeln(F("[] Starting..."));

  //ESP32PWM::allocateTimer(0);
	//ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  DroneWire::setup();

  // define root macro
  DEM_MACRO * root = dem.createMacro("root");
  DEM_INSTRUCTION_COMPILED instr;
  if (dem.compileLine(PSTR("AL \"/config.txt\""), &instr)) {
    root->commands->add(instr);
  } else {
    Log.errorln("Invalid command");
  }

  dem.compileLine(PSTR("FC \"/config.txt\""), &instr);
  root->commands->add(instr);
  dem.compileLine(PSTR("AL \"/main.txt\""), &instr);
  root->commands->add(instr);
  // now execute main
  dem.compileLine(PSTR("FC \"/main.txt\""), &instr);
  root->commands->add(instr);

  // prep execution of root
  DEM_CALLSTACK_ENTRY cse;
  cse.i=0;
  cse.macro = root;
  cse.continuation = false;
  dem.callStackPush(cse);

  // TODO: move to DEM
  //dmm.loadConfiguration();

  // load WIFI Configuration
  wifiManager.loadConfiguration();
  wifiManager.start(dmm);


  OTAMgr.init( dmm.hostname() );
  OTAMgr.onEvent = handleOTAEVent;

  MDNS.addService("http","tcp",80);

  #ifdef INC_WEB_SERVER
  setupWebServer();
  #endif

  // scan I2C buses
  DroneWire::scanAll();

  // Complete Setup of all modules
  //TODO: implement in DEM
  /*
  Log.noticeln(F("[] Completing module setup..."));
  dmm.setupModules();

  Log.noticeln(F("[] Setup done"));
  */

  // redirect logging to serial
  logFile.close();
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  //digitalWrite(PIN_LED, LOW);
}

long loopTime;
long testTimer;
uint8_t modid;

void loop() {
  loopTime = millis();

  dmm.watchdog();

  digitalWrite(PIN_LED, (WiFi.status() != WL_CONNECTED));

  if (!OTAMgr.isUpdating) {
    dmm.loopModules();

    dlm.processChannels();

    dem.execute();
  } else {
    digitalWrite(PIN_LED, HIGH);
  }

  OTAMgr.loop();
}
