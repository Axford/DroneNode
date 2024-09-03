#include "DroneSystem.h"
#include "DroneLogger.h"
#include "esp_pm.h"

static esp_pm_config_esp32_t pm_config = {
  .max_freq_mhz =240,// CONFIG_EXAMPLE_MAX_CPU_FREQ_MHZ,
  .min_freq_mhz = 10,//CONFIG_EXAMPLE_MIN_CPU_FREQ_MHZ,
  .light_sleep_enable = true
};

// ----------------------------------------------------------------------------
// protected
// ----------------------------------------------------------------------------

void DroneSystem::configurePin(uint8_t pin, uint8_t capabilities) {
  _pins[pin].capabilities = capabilities;
  _pins[pin].state = DRONE_SYSTEM_PIN_STATE_AVAILABLE;
}

// ----------------------------------------------------------------------------
// public
// ----------------------------------------------------------------------------

DroneSystem::DroneSystem() : _server(80), dfs(this), _fsEditor(LITTLEFS, _doLoop) {
  _motherboardVersion = 0;
  _SDAvailable = false;

  // create and give SPI Sempahore
  _xSPISemaphore = xSemaphoreCreateBinary();
  //xSemaphoreGive(xSPISemaphore);

  _doLoop = true;

  _serialCommandLen = 0;

  // init serial ports
  for (uint8_t i=0; i<DRONE_SYSTEM_SERIAL_PORTS; i++) {
    _serialPorts[i].state = DRONE_SYSTEM_SERIAL_PORT_STATE_INACTIVE;
    _serialPorts[i].module = NULL;
  }

  // init pins
  // set all to unavailble to start
  for (uint8_t i=0; i<DRONE_SYSTEM_PINS; i++) {
    _pins[i].state = DRONE_SYSTEM_PIN_STATE_UNAVAILABLE;
    _pins[i].capabilities = 0;
    _pins[i].strip = NULL;
    _pins[i].stripIndex = 0;
  }

  // now set specific capabilities

  // serial 0
  configurePin(1, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_SERIAL);
  configurePin(3, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_SERIAL);

  // serial 1
  configurePin(12, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_SERIAL);
  configurePin(13, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_SERIAL);

  // serial 2
  configurePin(17, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_SERIAL);
  configurePin(16, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_SERIAL);

  // SD chip select (for RFM69)
  configurePin(5, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT);


  // General IO, ADC1
  configurePin(32, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_ANALOG);
  configurePin(33, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_ANALOG);

  // input only, ADC1
  configurePin(34, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_ANALOG);
  configurePin(35, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_ANALOG);

  // LED
  configurePin(2, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_LED);

  // General IO, ADC2
  configurePin(14, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_ANALOG);
  configurePin(15, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_ANALOG);
  configurePin(4, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_ANALOG);
  configurePin(25, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_ANALOG);
  configurePin(26, DRONE_SYSTEM_PIN_CAP_INPUT | DRONE_SYSTEM_PIN_CAP_OUTPUT | DRONE_SYSTEM_PIN_CAP_ANALOG);

  // enable light sleep
  esp_pm_configure(&pm_config);
}


boolean DroneSystem::requestSerialPort(uint8_t port, DroneModule* module) {
  // check in range
  if (port < 0 || port >= DRONE_SYSTEM_SERIAL_PORTS ) {
    Log.errorln("[ds.rSP] Port out of range %u", port);
    return false;
  }

  // check port available
  if (_serialPorts[port].state < DRONE_SYSTEM_SERIAL_PORT_STATE_ACTIVE_MODULE ) {
    // module request for port 0?
    if (port == 0 && module != NULL) {
      // pins already registered at startup
      // disable logging to serial
      Log.noticeln(F("[ds.rSP] Silent log"));
      Log.setLevel(LOG_LEVEL_SILENT);
    } else {
      // register pins
      boolean registered = false;

      switch(port) {
        case 0:
          registered = requestPin(PIN_SERIAL0_TX, DRONE_SYSTEM_PIN_CAP_SERIAL, module) && requestPin(PIN_SERIAL0_RX, DRONE_SYSTEM_PIN_CAP_SERIAL, module);
          break;
        case 1:
          registered = requestPin(PIN_SERIAL1_TX, DRONE_SYSTEM_PIN_CAP_SERIAL, module) && requestPin(PIN_SERIAL1_RX, DRONE_SYSTEM_PIN_CAP_SERIAL, module);
          break;
        case 2:
          registered = requestPin(PIN_SERIAL2_TX, DRONE_SYSTEM_PIN_CAP_SERIAL, module) && requestPin(PIN_SERIAL2_RX, DRONE_SYSTEM_PIN_CAP_SERIAL, module);
          break;
      }

      if (!registered) {
        Log.errorln("[ds.rSP] Serial port pins unavailable %u", port);
        return false;
      }
    }

    if (module == NULL) {
      // builtin registration
      _serialPorts[port].state = DRONE_SYSTEM_SERIAL_PORT_STATE_ACTIVE_BUILTIN;
    } else {
      _serialPorts[port].state = DRONE_SYSTEM_SERIAL_PORT_STATE_ACTIVE_MODULE;
    }
    _serialPorts[port].module = module;

    return true;

  } else {
    Log.errorln("[ds.rSP] Port unavailable %u", port);
    return false;
  }
}


boolean DroneSystem::requestPin(uint8_t pin, uint8_t capabilities, DroneModule* module) {
  // check in range
  if (pin < 0 || pin >= DRONE_SYSTEM_PINS ) {
    Log.errorln("[ds.rP] Pin out of range %u", pin);
    return false;
  }

  // check available
  if (_pins[pin].state == DRONE_SYSTEM_PIN_STATE_AVAILABLE) {
    // check pin has requested capabilities
    if ((_pins[pin].capabilities & capabilities) == capabilities) {
      _pins[pin].state = DRONE_SYSTEM_PIN_STATE_ACTIVE;
      _pins[pin].module = module;
      return true;
    } else {
      Log.errorln("[ds.rP] Pin does not have requested capabilities %u, %u != %u", pin, _pins[pin].capabilities, capabilities);
      return false;
    }

  } else {
    Log.errorln("[ds.rP] Pin unavailable %u", pin);
    return false;
  }
}


NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>* DroneSystem::requestStrip(uint8_t pin, uint8_t pixels, DroneModule* module) {

  // if there is an existing strip on this pin then return it
  if (_pins[pin].strip) {
    return _pins[pin].strip;
  } else if (requestPin(pin, DRONE_SYSTEM_PIN_CAP_OUTPUT, module)) {
    // create new strip
    _pins[pin].strip = new NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>(pixels, pin);
    _pins[pin].strip->Begin();
    _pins[pin].strip->Show();
    _pins[pin].strip->SetBrightness(25);

    return _pins[pin].strip;
  };
  return NULL;
}

void DroneSystem::setStripFirstPixel(uint8_t pin, uint8_t index) {
  _pins[pin].stripIndex = index;
}

uint8_t DroneSystem::getStripFirstPixel(uint8_t pin) {
  return _pins[pin].stripIndex;
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
  // see if config.ini exists, if not create it
  if (!LITTLEFS.exists("/config.ini")) {
    File file = LITTLEFS.open("/config.ini", FILE_WRITE);
    if(!file){
        Log.errorln("[] Failed to open config.ini for writing");
        return;
    }
    file.println("node= 1");
    file.println("[Management = 1]");
    file.println("  name =setMe");
    file.println("  publish =hostname, IP");

    file.println("[UDPTelemetry = 2]");

    file.close();
  }
}


void DroneSystem::createSafeModeScript() {
  // TODO - adapt to use FS class for file handle
  // see if safeMode.ini exists, if not create it
  if (!LITTLEFS.exists("/safeMode.ini")) {
    File file = LITTLEFS.open("/safeMode.ini", FILE_WRITE);
    if(!file){
        Log.errorln("[] Failed to open config.ini for writing");
        return;
    }
    file.println("node= 1");
    file.println("[Management = 1]");
    file.println("  name =safeMode");
    file.println("  publish =hostname, IP");

    file.println("[UDPTelemetry = 2]");

    file.close();
  }
}


void DroneSystem::servePinInfo(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  response->addHeader("Server","ESP Async Web Server");
  response->print("[");

  uint8_t total = 0;
  for (uint8_t i=0; i<DRONE_SYSTEM_PINS; i++) {
    if (_pins[i].state > 0) {
      if (total > 0 ) response->print(",");
      total++;

      response->print("{");
      response->printf("\"id\":%u", i);
      response->printf(",\"state\":%u", _pins[i].state );
      
      response->printf(",\"output\":%u", ((_pins[i].capabilities & DRONE_SYSTEM_PIN_CAP_OUTPUT) > 0) ? 1 : 0);
      response->printf(",\"input\":%u", ((_pins[i].capabilities & DRONE_SYSTEM_PIN_CAP_INPUT) > 0) ? 1 : 0);
      response->printf(",\"analog\":%u", ((_pins[i].capabilities & DRONE_SYSTEM_PIN_CAP_ANALOG) > 0) ? 1 : 0);
      response->printf(",\"serial\":%u", ((_pins[i].capabilities & DRONE_SYSTEM_PIN_CAP_SERIAL) > 0) ? 1 : 0);
      response->printf(",\"LED\":%u", ((_pins[i].capabilities & DRONE_SYSTEM_PIN_CAP_LED) > 0) ? 1 : 0);
      
      response->printf(",\"strip\":%u", (_pins[i].strip >  0) ? 1 : 0);
      
      if (_pins[i].module) {
        response->printf(",\"module\": \"%u: %s\"", _pins[i].module->id(), _pins[i].module->getName());
      }
      response->print("}");
    }
  }

  response->print("]");

  //send the response last
  request->send(response);
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

  // pins
  _server.on("/pins", HTTP_GET, [&](AsyncWebServerRequest *request){
    _doLoop = false;
    servePinInfo(request);
    _doLoop = true;
  });

  // I2C scan info
  _server.on("/i2c", HTTP_GET, [&](AsyncWebServerRequest *request) {
    _doLoop = false;
    DroneWire::serveScanAllInfo(request);
    _doLoop = true;
  });

  // DEM handlers
  /*
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
  */

  
  _server.onNotFound([](AsyncWebServerRequest *request){
    if (request->method() == HTTP_OPTIONS) {
      request->send(200);
    } else {
      request->send(404, "application/json", "{\"message\":\"Not found\"}");
    }
  });
  

  _fsEditor.httpuser = "admin";
  _fsEditor.httppassword = "admin";
  _fsEditor.xSPISemaphore = _xSPISemaphore;
  _fsEditor.configureWebServer(_server);
  

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");

  //_server.serveStatic("/", LITTLEFS, "/").setDefaultFile("index.htm");

  _server.begin();
}


void DroneSystem::startInSafeMode() {
  Log.warningln(F("[] Prep SAFE start..."));
  dem->loadConfiguration("/safeMode.ini");
}


void DroneSystem::start() {
  Log.warningln(F("[] Prep NORMAL start..."));
  dem->loadConfiguration("/config.ini");
}


void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
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
  requestSerialPort(0, NULL);
  Serial.begin(115200);
  while(!Serial) {  }

  // Init log
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  delay(2500); // to allow serial to reconnect after programming

  Log.noticeln(F("[] Starting..."));

  // Init DroneWire
  Log.noticeln(F("[] Init DroneWire..."));
  DroneWire::setup();

  // I2C scan
  //Log.noticeln(F("[] I2C bus scan..."));
  //DroneWire::scanAll();

  // Detect motherboard version
  detectMotherboardVersion();
  Log.noticeln(F("[] Motherboard v%u"), _motherboardVersion);

  // Setup status LED
  dled = new DroneLED(this);

  // init DroneLogger
  if (_motherboardVersion >= 4) {
    // ensure pull ups on SPI pins
    pinMode(23,INPUT_PULLUP); 

    DroneLog.begin();
    DroneLog.enable();
  }

  /*
  SD CARD DEBUGGING
  */

  Log.noticeln("[] Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("  initialization failed!");
  } else {
    _SDAvailable = true;
    Log.noticeln("   initialization done.");
  }
  
  if (_SDAvailable) {
    // Create/Open file 
    File myFile = SD.open("/test.txt", FILE_WRITE);
    
    // if the file opened okay, write to it:
    if (myFile) {
      Log.noticeln("   Writing to file...");
      // Write to file
      myFile.println("Testing text 1, 2 ,3...");
      myFile.close(); // close the file
      Log.noticeln("   Done.");
    }
    // if the file didn't open, print an error:
    else {
      Log.errorln("error opening test.txt");
    }

    // print contents
    File root;
    root = SD.open("/");

    //printDirectory(root, 0);

    Log.noticeln("   done!");
  }
  

  // Determine and setup filesystem
  Log.noticeln(F("[] Init DroneFS..."));
  dfs.setup();

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
  dmm = new DroneModuleManager(dlm, LITTLEFS);
  // TODO - refactor to use FS class
  dem = new DroneExecutionManager(this, _logFile);

  // allocate PWM timers
  //ESP32PWM::allocateTimer(0);
	//ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  // TODO refactor FS class
  // switch to logging to startup.log file on flash if not in safeMode
  if (!dem->safeMode() && false) {
    Log.noticeln(F("[] Opening startup.log..."));
    _logFile = LITTLEFS.open("/startup.log", FILE_WRITE);

    Log.noticeln(F("[] Sending log to startup.log..."));
    Log.begin(LOG_LEVEL_VERBOSE, &_logFile);
  }

  // prep the startup scripts (will not execute until loop begins)
  if (dem->safeMode()) {
    startInSafeMode();
  } else {
    start();
  }

  dem->completeSetup();

  // attempt VPN
  //Husarnet.selfHostedSetup(husarnetDashboardURL);
  //Husarnet.join(husarnetJoinCode, (const char*)dmm->hostname().c_str());
  //Husarnet.start();

  // TODO - handle update events
  //_OTAMgr->onEvent = handleOTAEVent;
  //_OTAMgr->init( dmm->hostname() );

  // flush and close _logFile
  if (_logFile) {
    _logFile.flush();
    _logFile.close();
  }

  // switch to serial logging, unless we have telemetry enabled
  if (_serialPorts[0].state == DRONE_SYSTEM_SERIAL_PORT_STATE_ACTIVE_MODULE) {
    Log.begin(LOG_LEVEL_SILENT, &Serial);
  } else {
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.noticeln("[] End of Setup");
  }
}


void DroneSystem::loop() {
  //long loopTime;

  // TODO - add an FPS timer
  //loopTime = millis();


  if (_wifiManager.isEnabled() && WiFi.status() == WL_CONNECTED) {
    dled->setState(DRONE_LED_STATE_RUNNING_WIFI);
  } else {
    dled->setState(DRONE_LED_STATE_RUNNING_NO_WIFI);
  }


  // serial command interface
  // disable if logging is silent.. indicates we're using the serial port for telemtry
  if (_serialPorts[0].state == DRONE_SYSTEM_SERIAL_PORT_STATE_ACTIVE_BUILTIN) {
    if (Serial.available()) {
      char c = Serial.read();
      Serial.print(c);

      if (c == '\n' || c == '\r') {
        // null terminate
        _serialCommand[_serialCommandLen] = 0;
        Serial.print("Executing: ");
        Serial.println(_serialCommand);

        // clear boot flag and restart
        if (strcmp(_serialCommand, "reset")==0) {
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

  if (_doLoop) {

    dmm->watchdog();

    yield();

    dmm->loopModules();

    yield();

    dlm->loop();

    yield();

    dem->processAddressQueue();


    //if (logFile) logFile.flush();
  } else {
    //dled->setState(DRONE_LED_STATE_UPDATING);
  }

  //_OTAMgr->loop();

  // show strips
  for (uint8_t i=0; i<DRONE_SYSTEM_PINS; i++) {
    if (_pins[i].strip) _pins[i].strip->Show();
  }

  // return SPI semaphore
  xSemaphoreGive( _xSPISemaphore );
}
