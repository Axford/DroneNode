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
#include "DroneLinkMsg.h"
#include "DroneLinkChannel.h"
#include "DroneLinkManager.h"
#include "DroneModule.h"
#include "DroneModuleManager.h"
#include "DroneWire.h"
#include "DroneExecutionManager.h"

#define INC_WEB_SERVER

// other libraries
#include "FS.h"
#include <LITTLEFS.h>

#define FORMAT_LITTLEFS_IF_FAILED true

#include <AsyncTCP.h>
#include <ESPmDNS.h>

#ifdef INC_WEB_SERVER
#include <ESPAsyncWebServer.h>
#include "WebFSEditor.h"
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

boolean doLoop = true;

WiFiManager wifiManager;

#ifdef INC_WEB_SERVER
AsyncWebServer server(80);
WebFSEditor fsEditor(LITTLEFS, doLoop);
#endif

AsyncEventSource events("/events");
OTAManager OTAMgr(&events);

DroneLinkManager *dlm;
DroneModuleManager *dmm;
DroneExecutionManager *dem;

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
  fsEditor.configureWebServer(server);

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

  //server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");
  server.serveStatic("/", LITTLEFS, "/").setDefaultFile("index.htm");

  server.begin();
}
#endif


void handleOTAEVent(OTAManagerEvent event, float progress) {
  if (event == start) {
    dmm->shutdown();
  } else if (event == progress) {
    Serial.print(F("OTA progress: "));  Serial.println(progress*100);
    dmm->onOTAProgress(progress);
  }
}

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


void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  doLoop = false;
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if(!root){
      Serial.println("- failed to open directory");
      return;
  }
  if(!root.isDirectory()){
      Serial.println(" - not a directory");
      return;
  }

  File file = root.openNextFile();
  while(file){
      if(file.isDirectory()){
          Serial.print("  DIR : ");

#ifdef CONFIG_LITTLEFS_FOR_IDF_3_2
          Serial.println(file.name());
#else
          Serial.print(file.name());
          time_t t= file.getLastWrite();
          struct tm * tmstruct = localtime(&t);
          Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n",(tmstruct->tm_year)+1900,( tmstruct->tm_mon)+1, tmstruct->tm_mday,tmstruct->tm_hour , tmstruct->tm_min, tmstruct->tm_sec);
#endif

          if(levels){
              listDir(fs, file.name(), levels -1);
          }
      } else {
          Serial.print("  FILE: ");
          Serial.print(file.name());
          Serial.print("  SIZE: ");

#ifdef CONFIG_LITTLEFS_FOR_IDF_3_2
          Serial.println(file.size());
#else
          Serial.print(file.size());
          time_t t= file.getLastWrite();
          struct tm * tmstruct = localtime(&t);
          Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n",(tmstruct->tm_year)+1900,( tmstruct->tm_mon)+1, tmstruct->tm_mday,tmstruct->tm_hour , tmstruct->tm_min, tmstruct->tm_sec);
#endif
      }
      file = root.openNextFile();
  }
  doLoop = true;
}


void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  Serial.begin(115200);
  while(!Serial) {  }

  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  delay(2500); // to allow serial to reconnect after programming

  /*
  // direct log output to file in SPIFFS
  Log.noticeln(F("[] Starting SPIFFS"));
  if (!SPIFFS.begin(true)) { // formatonfail=true
    Log.errorln(F("[] Unable to begin SPIFFS"));
  }
  Log.noticeln(F("  totalBytes %d"), SPIFFS.totalBytes());
  Log.noticeln(F("  usedBytes %d"), SPIFFS.usedBytes());

  File logFile = SPIFFS.open("/startup.log", FILE_WRITE);
  */

  if(!LITTLEFS.begin(FORMAT_LITTLEFS_IF_FAILED)){
    Log.errorln("[] LITTLEFS Mount Failed");
    delay(1000);
    // Should be formatted and working after a reboot
    ESP.restart();
  }

  // list filesystem
  listDir(LITTLEFS, "/", 0);


  File logFile = LITTLEFS.open("/startup.log", FILE_WRITE);

  // switch to logging to startup.log file on spiffs
  Log.begin(LOG_LEVEL_VERBOSE, &logFile);
  Log.noticeln(F("[] Starting..."));

  DroneWire::setup();

  // create core objects
  dlm = new DroneLinkManager();
  dmm = new DroneModuleManager(dlm);
  dem = new DroneExecutionManager(dmm, dlm, LITTLEFS);

  //ESP32PWM::allocateTimer(0);
	//ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  if (dem->safeMode()) {
    Log.warningln(F("[] Starting in SAFE mode..."));
    // attempt to load and run safeMode script
    // define root macro
    DEM_MACRO * safeMode = dem->createMacro("safeMode");
    DEM_INSTRUCTION_COMPILED instr;
    if (dem->compileLine(PSTR("load \"/safeMode.txt\""), &instr))
      safeMode->commands->add(instr);
    if (dem->compileLine(PSTR("run \"/safeMode.txt\""), &instr))
      safeMode->commands->add(instr);

    // prep execution of safeMode
    DEM_CALLSTACK_ENTRY cse;
    cse.i=0;
    cse.macro = safeMode;
    cse.continuation = false;
    dem->callStackPush(cse);

  } else {
    Log.warningln(F("[] Starting in NORMAL mode..."));

    // prep and run normal boot process
    // define root macro
    DEM_MACRO * root = dem->createMacro("root");
    DEM_INSTRUCTION_COMPILED instr;
    if (dem->compileLine(PSTR("load \"/config.txt\""), &instr))
      root->commands->add(instr);
    dem->compileLine(PSTR("run \"/config.txt\""), &instr);
    root->commands->add(instr);
    dem->compileLine(PSTR("load \"/main.txt\""), &instr);
    root->commands->add(instr);
    // now execute main
    dem->compileLine(PSTR("run \"/main.txt\""), &instr);
    root->commands->add(instr);

    // prep execution of root
    DEM_CALLSTACK_ENTRY cse;
    cse.i=0;
    cse.macro = root;
    cse.continuation = false;
    dem->callStackPush(cse);
  }

  WiFi.disconnect();


  WiFi.mode(WIFI_AP_STA);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  String hostname = "Drone";
  for (uint8_t i=3; i<6; i++) {
    hostname += String(mac[i], HEX);
  }

  //WiFi.softAP(dmm->hostname().c_str());
  WiFi.softAP(hostname.c_str());

  // load WIFI Configuration
  wifiManager.loadConfiguration(LITTLEFS);
  wifiManager.start();

  OTAMgr.onEvent = handleOTAEVent;
  OTAMgr.init( dmm->hostname() );

  MDNS.addService("http","tcp",80);

  #ifdef INC_WEB_SERVER
  setupWebServer();
  #endif

  // scan I2C buses
  DroneWire::scanAll();

  // redirect logging to serial
  logFile.close();
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  // start core task
  /*
  xTaskCreatePinnedToCore(
    coreTask,        //Function to implement the task
    "coreTask",            //Name of the task
    16000,                   //Stack size in words
    0,                   //Task input parameter
    2,           //Priority of the task
    &_coreTask,                 //Task handle.
    1);              //Core where the task should run
*/

  //digitalWrite(PIN_LED, LOW);
}


long loopTime;
long testTimer;
uint8_t modid;
char serialCommand[30];
uint8_t serialCommandLen = 0;

void loop() {
  loopTime = millis();

  digitalWrite(PIN_LED, (WiFi.status() != WL_CONNECTED));

  // serial command interface
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print(c);

    if (c == '\n' || c == '\r') {
      // null terminate
      serialCommand[serialCommandLen] = 0;
      Serial.print("Executing: ");
      Serial.println(serialCommand);
      if (strcmp(serialCommand, "execute")==0) {
        Serial.println("restarting");
        dem->setBootStatus(DEM_BOOT_SUCCESS);
        dmm->restart();
      }

    } else {
      serialCommand[serialCommandLen] = c;
      if (serialCommandLen < 29 ) serialCommandLen++;
    }
  }

  if (!OTAMgr.isUpdating && doLoop) {
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

  OTAMgr.loop();
}
