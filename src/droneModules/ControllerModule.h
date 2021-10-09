/*

Drive an OLED module based on SSD1306 driver

display.begin(SSD1306_SWITCHCAPVCC, 0x3C)

*/
#ifndef CONTROLLER_MODULE_H
#define CONTROLLER_MODULE_H
#include "Arduino.h"
#include "ustd_platform.h"
#include "ustd_functional.h"

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"
#include "LinkedList.h"

#include "SSD1306Wire.h"


#define CONTROLLER_OLED_I2C_ADDRESS  0x3c

// pubs


#define CONTROLLER_PARAM_LEFT        8   // channel for left joystick
#define CONTROLLER_PARAM_LEFT_E      0

#define CONTROLLER_PARAM_RIGHT       9   // channel for right joystick
#define CONTROLLER_PARAM_RIGHT_E     1

#define CONTROLLER_PARAM_ENTRIES     2


// subs

#define CONTROLLER_SUBS              0

// strings
static const char CONTROLLER_STR_CONTROLLER[] PROGMEM = "Controller";


enum CONTROLLER_LABEL_STATE {
    CONTROLLER_LABEL_NOT_NEEDED,
    CONTROLLER_LABEL_NEEDED,
    CONTROLLER_LABEL_REQUESTED,
    CONTROLLER_LABEL_RECEIVED
};


//typedef ustd::function<void(void*, boolean)> ManageMenuHandler;

//typedef ustd::function<void(void*)> SelectMenuHandler;


struct CONTROLLER_MENU_ITEM {
  uint8_t menu;
  char name[16];
};


struct CONTROLLER_MENU_STATE {
  //uint8_t id;  // menu id - implicit
  uint8_t selected; // which item was selected
  uint8_t backTo;  // which menu item to go to if the left button is pushed
  char * name;  // menu name
  IvanLinkedList::LinkedList<CONTROLLER_MENU_ITEM> items;
  //ManageMenuHandler manageHandler;
  //SelectMenuHandler selectHandler;
};

enum CONTROLLER_MENUS {
  CONTROLLER_MENU_ROOT,
  CONTROLLER_MENU_MAIN,
  CONTROLLER_MENU_START,
  CONTROLLER_MENU_CREATE
};

// axis indices
#define CONTROLLER_AXIS_LEFT_X   0
#define CONTROLLER_AXIS_LEFT_Y   1
#define CONTROLLER_AXIS_LEFT_Z   2
#define CONTROLLER_AXIS_LEFT_B   3
#define CONTROLLER_AXIS_RIGHT_X  4
#define CONTROLLER_AXIS_RIGHT_Y  5
#define CONTROLLER_AXIS_RIGHT_Z  6
#define CONTROLLER_AXIS_RIGHT_B  7

// class
class ControllerModule:  public I2CBaseModule {
protected:
  float _axes[8];
  boolean _neutral[8];  // set true if entered neutral deadband, do provide hysterisis for menus

  unsigned long _syncMenusTimer;

  boolean _isBound;

  CONTROLLER_MENUS _menu;  // active menu

  CONTROLLER_MENU_STATE _menus[sizeof(CONTROLLER_MENUS)];

  int _scroll;  // index of first item shown on screen i.e. top

  SSD1306Wire *_display;
  DroneLinkMsg _queryMsg;
public:

  ControllerModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm);
  ~ControllerModule();

  void doReset();

  void doShutdown();

  void handleLinkMessage(DroneLinkMsg *msg);

  void loadConfiguration(JsonObject &obj);

  void setMenuItem(uint8_t menu, uint8_t item, char* name, uint8_t nextMenu);

  void publishEntry(uint8_t i);

  void manageRoot(boolean syncMenu);
  void manageMain(boolean syncMenu);
  void manageStart(boolean syncMenu);
  void manageCreate(boolean syncMenu);

  void drawMenu();

  void loop();


};

#endif
