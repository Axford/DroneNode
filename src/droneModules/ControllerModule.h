/*

Drive an OLED module based on SSD1306 driver

display.begin(SSD1306_SWITCHCAPVCC, 0x3C)

*/
#ifndef CONTROLLER_MODULE_H
#define CONTROLLER_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "SSD1306Wire.h"

#define CONTROLLER_OLED_I2C_ADDRESS  0x3c

// pubs
#define CONTROLLER_PARAM_ENTRIES           0

// subs
#define CONTROLLER_SUB_SUB1          10
#define CONTROLLER_SUB_SUB1_ADDR     11
#define CONTROLLER_SUB_SUB1_E        0

#define CONTROLLER_SUB_SUB2          12
#define CONTROLLER_SUB_SUB2_ADDR     13
#define CONTROLLER_SUB_SUB2_E        1

#define CONTROLLER_SUB_SUB3          14
#define CONTROLLER_SUB_SUB3_ADDR     15
#define CONTROLLER_SUB_SUB3_E        2

#define CONTROLLER_SUB_SUB4          16
#define CONTROLLER_SUB_SUB4_ADDR     17
#define CONTROLLER_SUB_SUB4_E        3

#define CONTROLLER_SUBS              4

#define CONTROLLER_NUM_LABELS        CONTROLLER_SUBS

// strings
static const char CONTROLLER_STR_CONTROLLER[] PROGMEM = "OLED";


enum CONTROLLER_LABEL_STATE {
    CONTROLLER_LABEL_NOT_NEEDED,
    CONTROLLER_LABEL_NEEDED,
    CONTROLLER_LABEL_REQUESTED,
    CONTROLLER_LABEL_RECEIVED
};

// class
class ControllerModule:  public I2CBaseModule {
protected:
  char _subLabels[CONTROLLER_NUM_LABELS][17];
  uint8_t _labelState[CONTROLLER_NUM_LABELS];
  unsigned long _lastDiscovery;
  SSD1306Wire *_display;
  DroneLinkMsg _queryMsg;
public:

  ControllerModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm);
  ~ControllerModule();

  void doReset();

  void onOTAProgress(float progress);

  void doShutdown();

  void handleLinkMessage(DroneLinkMsg *msg);

  void loadConfiguration(JsonObject &obj);

  void publishEntry(uint8_t i);

  virtual void loop();


};

#endif
