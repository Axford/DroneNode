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
#include "DroneFS.h"

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
//#include <ArduinoJson.h>

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

DroneFS dfs;
DroneLinkManager *dlm;
DroneModuleManager *dmm;
DroneExecutionManager *dem;

File logFile;

const char* QUERY_PARAM_APMODE = "APMode";
const char* QUERY_PARAM_SSID = "ssid";
const char* QUERY_PARAM_PASSWORD = "password";

// semaphore used to avoid conflicts on SPI bus between filesystem and telemetry radio
// caused by async webserver
SemaphoreHandle_t xSPISemaphore;


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
    dmm->shutdown();
  } else if (event == progress) {
    Serial.print(F("OTA progress: "));  Serial.println(progress*100);
    dmm->onOTAProgress(progress);
  }
}


void handleDLMEvent( DroneLinkManagerEvent event, float progress) {
  if (event == DRONE_LINK_MANAGER_FIRMWARE_UPDATE_START) {
    dmm->updateStarting();

  } else if (event == DRONE_LINK_MANAGER_FIRMWARE_UPDATE_END) {

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


void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  Serial.begin(115200);
  while(!Serial) {  }

  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  delay(2500); // to allow serial to reconnect after programming

  // create and give SPI Sempahore
  xSPISemaphore = xSemaphoreCreateBinary();
  //xSemaphoreGive(xSPISemaphore);


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


  logFile = LITTLEFS.open("/startup.log", FILE_WRITE);

  // switch to logging to startup.log file on flash
  Serial.println(F("[] Sending log to startup.log..."));
  //Log.begin(LOG_LEVEL_VERBOSE, &logFile);
  //Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.noticeln(F("[] Starting..."));

  DroneWire::setup();

  //WiFi.disconnect();  // moved to WiFiManager
  // WiFi.mode(WIFI_AP_STA); // moved to WiFiManager

  // load WIFI Configuration and enable during startup
  Log.noticeln(F("[] Load WiFi config..."));
  wifiManager.loadConfiguration(LITTLEFS);
  wifiManager.enable();
  wifiManager.start();

  MDNS.addService("http","tcp",80);

  #ifdef INC_WEB_SERVER
  setupWebServer();
  #endif

  // create core objects
  Log.noticeln(F("[] Init DroneLink core..."));
  dlm = new DroneLinkManager(&wifiManager, &dfs);
  dlm->onEvent = handleDLMEvent;
  dmm = new DroneModuleManager(dlm);
  dem = new DroneExecutionManager(dmm, dlm, LITTLEFS, logFile);

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

  OTAMgr.onEvent = handleOTAEVent;
  OTAMgr.init( dmm->hostname() );


  // scan I2C buses
  DroneWire::scanAll();

  // redirect logging to serial
  logFile.flush();
  logFile.close();

  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.noticeln(F("[] Startup complete"));

  // setup FS
  dfs.setup();

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

  if (wifiManager.isEnabled()) {
    digitalWrite(PIN_LED, (WiFi.status() != WL_CONNECTED));
  } else {
    digitalWrite(PIN_LED, LOW);
  }


  // serial command interface
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print(c);

    if (c == '\n' || c == '\r') {
      // null terminate
      serialCommand[serialCommandLen] = 0;
      Serial.print("Executing: ");
      Serial.println(serialCommand);

      // clear boot flag and restart
      if (strcmp(serialCommand, "execute")==0) {
        Serial.println("restarting");
        dem->setBootStatus(DEM_BOOT_SUCCESS);
        dmm->restart();
      }

      // format filesystem
      if (strcmp(serialCommand, "format")==0) {
        Serial.println("Formatting");
        LITTLEFS.format();
        dmm->restart();
      }

      // traceroute
      if (strncmp(serialCommand, "trace", 5)==0) {
        // read node address
        uint8_t destNode = atoi(&serialCommand[6]);

        Log.noticeln("Traceroute to %u", destNode);
        dlm->generateTraceroute(destNode);
      }

      serialCommandLen = 0;

    } else {
      serialCommand[serialCommandLen] = c;
      if (serialCommandLen < 29 ) serialCommandLen++;
    }
  }

  if (!OTAMgr.isUpdating && doLoop) {

    // take SPI semaphore
    //Serial.println("take 1");
    //xSemaphoreTake( xSPISemaphore, portMAX_DELAY );
    //Serial.println("taken 1");

    dmm->watchdog();

    yield();

    //Log.noticeln("[] lM");
    dmm->loopModules();

    yield();

    //Log.noticeln("[] pC");
    dlm->loop();

    yield();

    //Log.noticeln("[] e");
    dem->execute();

    //if (logFile) logFile.flush();
  } else {
    digitalWrite(PIN_LED, HIGH);
  }

  // return SPI semaphore
  //Serial.println("give 1");
  xSemaphoreGive( xSPISemaphore );

  OTAMgr.loop();
}
