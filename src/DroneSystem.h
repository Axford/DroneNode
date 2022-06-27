/*

## DroneSystem

DroneSystem is the root of all Drone objects, keeping the main script as simple as possible.  System object is passed to all modules as means to access underlying hardware and services.

* Hosts all the other Drone core objects (module mgr, execution mgr, etc)
* HAL and underlying services
  * Status LEDs (neopixel strings)
  * filesystem (flash vs SD)
  * IO pin registration
  * interrupt routines
  * timers?
  * sleep system?
  * cpu freq?
  * Logging
  * Serial ports?
  * I2C multiplexer
  * SPI conflicts?
  * memory usage
  * cpu utilisation
  * firmware OTA updates
  * serial interface (for manual config/overrides)

*/

#ifndef DRONE_SYSTEM_H
#define DRONE_SYSTEM_H

// arduino core
#include "Arduino.h"

// drone core
#include "DroneLinkMsg.h"
#include "DroneLinkChannel.h"
#include "DroneLinkManager.h"
#include "DroneModule.h"
#include "DroneModuleManager.h"
#include "DroneWire.h"
#include "DroneExecutionManager.h"
#include "DroneFS.h"
#include "DroneLED.h"
#include "pinConfig.h"

// file system
#include "FS.h"
#include <LITTLEFS.h>

// web services
#include "WiFiManager.h"
#include <ESPAsyncWebServer.h>

// other
#include <ESP32Servo.h>
#include <ArduinoLog.h>

class DroneSystem {
protected:
  // motherboard version
  uint8_t _motherboardVersion;

  // handle to logFile
  File _logFile;

  // semaphore used to avoid conflicts on SPI bus between filesystem and telemetry radio
  // caused by async webserver
  SemaphoreHandle_t _xSPISemaphore;

  WiFiManager _wifiManager;

  boolean _doLoop;

  char _serialCommand[30];
  uint8_t _serialCommandLen;

  // web server
  AsyncWebServer _server;

public:

  DroneFS dfs;
  DroneLinkManager *dlm;
  DroneModuleManager *dmm;
  DroneExecutionManager *dem;
  DroneLED* dled;

  DroneSystem();

  void detectMotherboardVersion();

  uint8_t motherboardVersion();

  void createDefaultConfig();

  // see if safeMode.txt exists, if not create it
  void createSafeModeScript();

  void setupWebServer();

  // startup using safeMode.txt
  void startInSafeMode();

  // normal startup from config.txt
  void start();

  void setup();

  void loop();
};


#endif
