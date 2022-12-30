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
 - Connect in web-browser
 - Upload config
 - Reboot ESP and watch terminal to ensure config is loaded correctly

*/

//#define CONFIG_ARDUINO_LOOP_STACK_SIZE 16384

// arduino core
#include <functional>
#include <Arduino.h>

#include <ArduinoLog.h>

// drone core
#include "DroneSystem.h"

// networking
#include <AsyncTCP.h>
#include <ESPmDNS.h>

#include "OTAManager.h"


DroneSystem ds;
OTAManager OTAMgr;


void handleOTAEVent(OTAManagerEvent event, float p) {
  if (event == start) {
    ds.dmm->shutdown();
    ds.dled->setState(DRONE_LED_STATE_UPDATING);
  } else if (event == progress) {
    //Log.noticeln(F("OTA progress: %f"), p*100);
    ds.dmm->onOTAProgress(p);
    ds.dled->loop();
  } else  {
    // end
    ds.dled->setState(DRONE_LED_STATE_RESTART);
  }
}


void handleDLMEvent( DroneLinkManagerEvent event, float progress) {
  if (event == DRONE_LINK_MANAGER_FIRMWARE_UPDATE_START) {
    //TODO
    //dmm->updateStarting();

  } else if (event == DRONE_LINK_MANAGER_FIRMWARE_UPDATE_END) {

  }
}



void setup() {

  disableCore0WDT();
  disableCore1WDT();
  disableLoopWDT();
  
  ds.setup();

  OTAMgr.init(ds.dmm->hostname());
  OTAMgr.onEvent = handleOTAEVent;
}


void loop() {
  
  if (!OTAMgr.isUpdating) {
    ds.loop();
  } else {
    ds.dled->loop();
  }

  OTAMgr.loop();
}
