/*

@type          Management
@inherits      Drone
@description   Provides overall system management and a pub/sub interface to the DroneModuleManager

*/
#ifndef MANAGEMENT_MODULE_H
#define MANAGEMENT_MODULE_H

#include "../DroneModule.h"


// published params
// @pub 8;c;1;hostname;Hostname of the device
#define MANAGEMENT_PARAM_HOSTNAME      8

// @pub 9;c;1;build;Git commit hash at the time the firmware was built
#define MANAGEMENT_PARAM_BUILD         9

// @pub 10;u8;1;reset;Set to 1 to trigger a reset
#define MANAGEMENT_PARAM_RESET         10

// @pub 11;u32;1;heap;Current size of heap (i.e. free memory)
#define MANAGEMENT_PARAM_HEAP          11

// @pub 12;u8;4;IP;IP address
#define MANAGEMENT_PARAM_IP            12

// @pub 13;u32;1;uptime;Uptime in seconds
#define MANAGEMENT_PARAM_UPTIME        13

// @pub 14;f;1;publishRate;Rate of messages published per second
#define MANAGEMENT_PARAM_PUBLISHRATE   14

// @pub 15;u32;1;choked;Number of times a channel queue has choked (rejected a msg becuse full)
#define MANAGEMENT_PARAM_CHOKED        15

// @pub 16;u8;1;discovery;Enable/disable node discovery process.  Set to 1 to enable, or 0 to disable.
#define MANAGEMENT_PARAM_DISCOVERY     16

// @pub 17;c;1;macro;Set to the path to a macro file (.txt file) to run the associated macro.  Note - the macro must be loaded first (e.g. via config.txt).
#define MANAGEMENT_PARAM_MACRO         17

// @pub 18;u8;1;wifi;Enable/disable wifi. Set to 1 to enable, or 0 to disable.
#define MANAGEMENT_PARAM_WIFI          18

// @pub 19;u32;1;sleep;How long to sleep in main loop - uses light sleep
#define MANAGEMENT_PARAM_SLEEP         19

// @pub 20;u32;1;cpu;CPU freq in MHz, valid values 240, 160, 80
#define MANAGEMENT_PARAM_CPU           20

#define MANAGEMENT_PARAM_HOSTNAME_E     0
#define MANAGEMENT_PARAM_BUILD_E        1
#define MANAGEMENT_PARAM_RESET_E        2
#define MANAGEMENT_PARAM_HEAP_E         3
#define MANAGEMENT_PARAM_IP_E           4
#define MANAGEMENT_PARAM_UPTIME_E       5
#define MANAGEMENT_PARAM_PUBLISHRATE_E  6
#define MANAGEMENT_PARAM_CHOKED_E       7
#define MANAGEMENT_PARAM_DISCOVERY_E    8
#define MANAGEMENT_PARAM_MACRO_E        9
#define MANAGEMENT_PARAM_WIFI_E         10
#define MANAGEMENT_PARAM_SLEEP_E        11
#define MANAGEMENT_PARAM_CPU_E          12

#define MANAGEMENT_PARAM_ENTRIES        13


// strings
static const char MANAGEMENT_STR_MANAGEMENT[] PROGMEM = "Management";

// class
class ManagementModule:  public DroneModule {
protected:
  unsigned long _lastRate;
public:

  ManagementModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void onParamWrite(DRONE_PARAM_ENTRY *param);

  virtual void setup();
  virtual void loop();

  uint8_t diagnosticDisplays();
  void drawDiagnosticDisplay(SSD1306Wire *display, uint8_t page);
};

#endif
