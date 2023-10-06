/*

## DroneSystem

DroneSystem is the root of all Drone objects, keeping the main script as simple as possible.  System object is passed to all modules as means to access underlying hardware and services.

* Hosts all the other Drone core objects (module mgr, execution mgr, etc)
* HAL and underlying services
  * Status LEDs (neopixel strings)
  * filesystem (flash vs SD)
  * IO pin registration
  * Serial port registration
  * interrupt routines
  * timers?
  * sleep system?
  * cpu freq?
  * Logging
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
#include <LittleFS.h>
#define LITTLEFS LittleFS
#include <SPI.h>
#include <SD.h>

// web services
#include "WiFiManager.h"
#include <ESPAsyncWebServer.h>
#include "WebFSEditor.h"

// other
#include <ESP32Servo.h>
#include <ArduinoLog.h>

// vpn support
//#include <Husarnet.h>


// ----------------------------------------------------------------------------
// VPN Credentials
// ----------------------------------------------------------------------------

const char* husarnetJoinCode = "fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/fgUY4RJhfxPzmnRVUZQtcN";
const char* husarnetDashboardURL = "default";

// ----------------------------------------------------------------------------
// Serial port mgmt
// ----------------------------------------------------------------------------

#define DRONE_SYSTEM_SERIAL_PORT_STATE_INACTIVE        0
#define DRONE_SYSTEM_SERIAL_PORT_STATE_ACTIVE_BUILTIN  1
#define DRONE_SYSTEM_SERIAL_PORT_STATE_ACTIVE_MODULE   2

struct DRONE_SYSTEM_SERIAL_PORT {
  uint8_t state;
  DroneModule* module;
};

#define DRONE_SYSTEM_SERIAL_PORTS  3   // 0..2


// ----------------------------------------------------------------------------
// IO pin mgmt
// ----------------------------------------------------------------------------

// states
#define DRONE_SYSTEM_PIN_STATE_UNAVAILABLE    0
#define DRONE_SYSTEM_PIN_STATE_AVAILABLE      1
#define DRONE_SYSTEM_PIN_STATE_ACTIVE         2

// bit masks for capabilities
#define DRONE_SYSTEM_PIN_CAP_OUTPUT     1
#define DRONE_SYSTEM_PIN_CAP_INPUT      2
#define DRONE_SYSTEM_PIN_CAP_ANALOG     4
#define DRONE_SYSTEM_PIN_CAP_SERIAL     8
#define DRONE_SYSTEM_PIN_CAP_LED        16

struct DRONE_SYSTEM_PIN {
  uint8_t state;
  uint8_t capabilities;
  DroneModule* module;
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> * strip; // if a strip is associated with this pin
  uint8_t stripIndex;  // index of first pixel
};

#define DRONE_SYSTEM_PINS   36    // 0..35


// ----------------------------------------------------------------------------
// DroneSystem
// ----------------------------------------------------------------------------
class DroneSystem {
protected:
  // motherboard version
  uint8_t _motherboardVersion;

  boolean _SDAvailable;  // true if SD card available

  // serial port mgmt
  DRONE_SYSTEM_SERIAL_PORT _serialPorts[DRONE_SYSTEM_SERIAL_PORTS];

  DRONE_SYSTEM_PIN _pins[DRONE_SYSTEM_PINS];

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
  WebFSEditor _fsEditor;

  void configurePin(uint8_t pin, uint8_t capabilities);

public:

  DroneFS dfs;
  DroneLinkManager *dlm;
  DroneModuleManager *dmm;
  DroneExecutionManager *dem;
  DroneLED* dled;

  DroneSystem();

  boolean requestSerialPort(uint8_t port, DroneModule* module);
  boolean requestPin(uint8_t pin, uint8_t capabilities, DroneModule* module);
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>* requestStrip(uint8_t pin, uint8_t pixels, DroneModule* module);
  void setStripFirstPixel(uint8_t pin, uint8_t index);
  // get index of first pixel for strip on pin
  uint8_t getStripFirstPixel(uint8_t pin);

  void detectMotherboardVersion();

  uint8_t motherboardVersion();

  void createDefaultConfig();

  // see if safeMode.txt exists, if not create it
  void createSafeModeScript();

  void servePinInfo(AsyncWebServerRequest *request);
  
  void setupWebServer();

  // startup using safeMode.txt
  void startInSafeMode();

  // normal startup from config.txt
  void start();

  void setup();

  void loop();
};


#endif
