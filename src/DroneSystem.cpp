#include "DroneSystem.h"

DroneSystem::DroneSystem() : _server(80) {
  _motherboardVersion = 0;

  // create and give SPI Sempahore
  _xSPISemaphore = xSemaphoreCreateBinary();
  //xSemaphoreGive(xSPISemaphore);

  _doLoop = true;

  _serialCommandLen = 0;
}


void DroneSystem::detectMotherboardVersion() {
  _motherboardVersion = DroneWire::getMultiplexerVersion();

  if (_motherboardVersion == 2) {
    // see if we are on a v4 board by looking for the compass chip
    // QMC5883L_I2C_ADDRESS  0x0D
    DroneWire::selectChannel(0);
    if (DroneWire::scanAddress(0x0D)) {
      _motherboardVersion = 4;
    }
  }
}


uint8_t DroneSystem::motherboardVersion() {
  return _motherboardVersion;
}


void DroneSystem::createDefaultConfig() {
  // TODO - adapt to use FS class for file handle
  // see if config.txt exists, if not create it
  if (!LITTLEFS.exists("/config.txt")) {
    File file = LITTLEFS.open("/config.txt", FILE_WRITE);
    if(!file){
        Log.errorln("[] Failed to open config.txt for writing");
        return;
    }
    file.println("node 1");
    file.println("Management.new 1");
    file.println("  name \"set me\"");
    file.println("  .publish \"hostname\"");
    file.println("  .publish \"IP\"");
    file.println(".done");
    file.println("UDPTelemetry.new 2");
    file.println("  port 8007");
    file.println("  broadcast 255 255 255 255");
    file.println(".done");
    file.println(".setup");

    file.close();
  }
}


void DroneSystem::createSafeModeScript() {
  // TODO - adapt to use FS class for file handle
  // see if safeMode.txt exists, if not create it
  if (!LITTLEFS.exists("/safeMode.txt")) {
    File file = LITTLEFS.open("/safeMode.txt", FILE_WRITE);
    if(!file){
        Log.errorln("[] Failed to open safeMode.txt for writing");
        return;
    }
    file.println("node 1");
    file.println("Management.new 1");
    file.println("  name \"safeMode\"");
    file.println("  .publish \"hostname\"");
    file.println("  .publish \"IP\"");
    file.println(".done");
    file.println("UDPTelemetry.new 2");
    file.println("  port 8007");
    file.println("  broadcast 255 255 255 255");
    file.println(".done");
    file.println(".setup");

    file.close();
  }
}


void DroneSystem::setupWebServer() {
  Log.noticeln(F("[] Setup web _server..."));

  // node info debug
  _server.on("/nodeInfo", HTTP_GET, [&](AsyncWebServerRequest *request){
    _doLoop = false;
    dlm->serveNodeInfo(request);
    _doLoop = true;
  });

  // channel info debug
  _server.on("/channelInfo", HTTP_GET, [&](AsyncWebServerRequest *request){
    _doLoop = false;
    dlm->serveChannelInfo(request);
    _doLoop = true;
  });

  // modules
  _server.on("/modules", HTTP_GET, [&](AsyncWebServerRequest *request){
    _doLoop = false;
    dmm->serveModuleInfo(request);
    _doLoop = true;
  });

  // DEM handlers
  _server.on("/macros", HTTP_GET, [&](AsyncWebServerRequest *request){
    _doLoop = false;
    dem->serveMacroInfo(request);
    _doLoop = true;
  });
  _server.on("/execution", HTTP_GET, [&](AsyncWebServerRequest *request){
    _doLoop = false;
    dem->serveExecutionInfo(request);
    _doLoop = true;
  });
  _server.on("/commands", HTTP_GET, [&](AsyncWebServerRequest *request){
    _doLoop = false;
    dem->serveCommandInfo(request);
    _doLoop = true;
  });

  /*
  _server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404);
  });
  */

  /*
  fsEditor.httpuser = "admin";
  fsEditor.httppassword = "admin";
  fsEditor.xSPISemaphore = xSPISemaphore;
  fsEditor.configureWebServer(_server);
  */

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

  _server.serveStatic("/", LITTLEFS, "/").setDefaultFile("index.htm");

  _server.begin();
}


void DroneSystem::startInSafeMode() {
  Log.warningln(F("[] Prep SAFE start..."));
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
}


void DroneSystem::start() {
  Log.warningln(F("[] Prep NORMAL start..."));

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


void DroneSystem::setup() {
  /*

  Startup process

  - Start serial and init log
  - Init DroneWire
  - Detect motherboard version (using I2C scans)
  - Setup status LED (on or white)
  - Determine and setup filesystem
  - Create startup.log
  - Create safeMode script
  - Init wifi
  - Create core objects
  - Allocate PWM timers
  - start (or startInSafeMode)
  - Flush startup log
  - Switch loggig to serial
  - Update status LED (on or blue)

  */

  // Start serial
  Serial.begin(115200);
  while(!Serial) {  }

  // Init log
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  delay(2500); // to allow serial to reconnect after programming

  Log.noticeln(F("[] Starting..."));

  // Init DroneWire
  Log.noticeln(F("[] Init DroneWire..."));
  DroneWire::setup();

  // Detect motherboard version
  detectMotherboardVersion();
  Log.noticeln(F("[] Motherboard v%u"), _motherboardVersion);

  // Setup status LED
  dled = new DroneLED(this);

  // Determine and setup filesystem
  Log.noticeln(F("[] Init DroneFS..."));
  dfs.setup();


  // TODO - move to FS class
  Log.noticeln(F("[] Mount filesystem..."));
  // passing true to .begin triggers a format if required
  if(!LITTLEFS.begin(true)){
    Log.errorln("[] LITTLEFS Mount Failed");
    dled->setState(DRONE_LED_STATE_ERROR);
    delay(3000);
    // Should be formatted and working after a reboot
    ESP.restart();
  }

  // TODO refactor FS class
  Log.noticeln(F("[] Opening startup.log..."));
  _logFile = LITTLEFS.open("/startup.log", FILE_WRITE);

  // switch to logging to startup.log file on flash
  Log.noticeln(F("[] Sending log to startup.log..."));
  Log.begin(LOG_LEVEL_VERBOSE, &_logFile);
  //Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  createDefaultConfig();
  createSafeModeScript();

  // load WIFI Configuration and enable during startup
  // TODO - refactor to use FS class
  Log.noticeln(F("[] Init WiFiManager..."));
  _wifiManager.loadConfiguration(LITTLEFS);
  _wifiManager.enable();
  _wifiManager.start();

  Log.noticeln(F("[] Init web server..."));
  setupWebServer();

  // create core objects
  Log.noticeln(F("[] Init DroneLink core..."));
  dlm = new DroneLinkManager(&_wifiManager, &dfs);
  // TODO - rework event handling
  //dlm->onEvent = handleDLMEvent;
  dmm = new DroneModuleManager(dlm);
  // TODO - refactor to use FS class
  dem = new DroneExecutionManager(this, _logFile);

  // allocate PWM timers
  //ESP32PWM::allocateTimer(0);
	//ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  // prep the startup scripts (will not execute until loop begins)
  if (dem->safeMode()) {
    startInSafeMode();
  } else {
    start();
  }

  // scan I2C buses
  // TODO - make this a serial or web interface function - no need todo on every boot
  //DroneWire::scanAll();

  // TODO - integrate OTA
  /*
  OTAMgr.onEvent = handleOTAEVent;
  OTAMgr.init( dmm->hostname() );
  */

  // flush and close _logFile
  _logFile.flush();
  _logFile.close();

  // switch to serial logging
  Log.begin(Log.getLevel(), &Serial);

  Log.noticeln(F("[] End of setup"));
}


void DroneSystem::loop() {
  //long loopTime;

  // TODO - add an FPS timer
  //loopTime = millis();

  // TODO - rework status LED
  if (_wifiManager.isEnabled() && WiFi.status() == WL_CONNECTED) {
    dled->setState(DRONE_LED_STATE_RUNNING_WIFI);
  } else {
    dled->setState(DRONE_LED_STATE_RUNNING_NO_WIFI);
  }


  // serial command interface
  // disable if logging is silent.. indicates we're using the serial port for telemtry
  if (Log.getLevel() != LOG_LEVEL_SILENT) {
    if (Serial.available()) {
      char c = Serial.read();
      Serial.print(c);

      if (c == '\n' || c == '\r') {
        // null terminate
        _serialCommand[_serialCommandLen] = 0;
        Serial.print("Executing: ");
        Serial.println(_serialCommand);

        // clear boot flag and restart
        if (strcmp(_serialCommand, "execute")==0) {
          Serial.println("restarting");
          dem->setBootStatus(DEM_BOOT_SUCCESS);
          dmm->restart();
        }

        // format filesystem
        if (strcmp(_serialCommand, "format")==0) {
          Serial.println("Formatting");
          LITTLEFS.format();
          dmm->restart();
        }

        // enable wifi
        if (strcmp(_serialCommand, "wifi")==0) {
          Serial.println("Enabling WiFi");
          dlm->enableWiFi();
        }

        // traceroute
        if (strncmp(_serialCommand, "trace", 5)==0) {
          // read node address
          uint8_t destNode = atoi(&_serialCommand[6]);

          Log.noticeln("Traceroute to %u", destNode);
          dlm->generateTraceroute(destNode);
        }

        _serialCommandLen = 0;

      } else {
        _serialCommand[_serialCommandLen] = c;
        if (_serialCommandLen < 29 ) _serialCommandLen++;
      }
    }
  }

  // TODO
  //if (!OTAMgr.isUpdating && doLoop) {
  if (_doLoop) {

    dmm->watchdog();

    yield();

    dmm->loopModules();

    yield();

    dlm->loop();

    yield();

    dem->execute();

    //if (logFile) logFile.flush();
  } else {
    // TODO - need an updating state
    //digitalWrite(PIN_LED, HIGH);
  }

  // return SPI semaphore
  xSemaphoreGive( _xSPISemaphore );

  //TODO
  //OTAMgr.loop();
}
