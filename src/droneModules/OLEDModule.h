/*

Drive an OLED module based on SSD1306 driver

display.begin(SSD1306_SWITCHCAPVCC, 0x3C)

*/
#ifndef OLED_MODULE_H
#define OLED_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "SSD1306Wire.h"

#define OLED_I2C_ADDRESS  0x3c

// pubs
#define OLED_PARAM_ENTRIES           0

// subs
#define OLED_SUB_SUB1          10
#define OLED_SUB_SUB1_ADDR     11
#define OLED_SUB_SUB1_E        0

#define OLED_SUB_SUB2          12
#define OLED_SUB_SUB2_ADDR     13
#define OLED_SUB_SUB2_E        1

#define OLED_SUB_SUB3          14
#define OLED_SUB_SUB3_ADDR     15
#define OLED_SUB_SUB3_E        2

#define OLED_SUB_SUB4          16
#define OLED_SUB_SUB4_ADDR     17
#define OLED_SUB_SUB4_E        3

#define OLED_SUBS              4

#define OLED_NUM_LABELS        OLED_SUBS

// strings
static const char OLED_STR_OLED[] PROGMEM = "OLED";


enum OLED_LABEL_STATE { notNeeded, needed, requested, received };

// class
class OLEDModule:  public I2CBaseModule {
protected:
  char _subLabels[OLED_NUM_LABELS][17];
  uint8_t _labelState[OLED_NUM_LABELS];
  unsigned long _lastDiscovery;
  SSD1306Wire *_display;
  DroneLinkMsg _queryMsg;
public:

  OLEDModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);
  ~OLEDModule();

  void doReset();

  void onOTAProgress(float progress);

  void doShutdown();

  void handleLinkMessage(DroneLinkMsg *msg);

  void publishEntry(uint8_t i);

  virtual void loop();


};

#endif
