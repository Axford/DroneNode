/*

Drive an OLED module based on SSD1306 driver

display.begin(SSD1306_SWITCHCAPVCC, 0x3C)

*/
#ifndef CONTROLLER_MODULE_H
#define CONTROLLER_MODULE_H
#include "Arduino.h"
#include <functional>

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"
#include "LinkedList.h"

#include "SSD1306Wire.h"


#define CONTROLLER_OLED_I2C_ADDRESS  0x3c


// pubs
#define CONTROLLER_PARAM_ENTRIES     (I2CBASE_PARAM_ENTRIES + 0)


// subs
#define CONTROLLER_SUBS              0

// strings
static const char CONTROLLER_STR_CONTROLLER[] PROGMEM = "Controller";



// class
class ControllerModule:  public I2CBaseModule {
protected:

  uint8_t _brightness;

  boolean _armed;  // true if we are sending control data

  int _scroll;  // index of first item shown on screen i.e. top
  float _spinner;

  SSD1306Wire *_display;
  DroneLinkMsg _sendMsg;
public:

  ControllerModule(uint8_t id, DroneSystem* ds);
  ~ControllerModule();

  void clear();
  void drawSpinner();

  void doReset();

  void doShutdown();

  void handleLinkMessage(DroneLinkMsg *msg);

  void setup();

  void loop();


};

#endif
