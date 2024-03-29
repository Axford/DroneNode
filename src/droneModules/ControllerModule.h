/*

@type          Controller
@inherits      I2CBase
@category      Logic
@description   Manage a universal remote control using an SSD1306 display as UI

@config >>>
[Controller= 10]
  name= "Controller"
  bus= 0
<<<

*/

/*
display.begin(SSD1306_SWITCHCAPVCC, 0x3C)
*/
#ifndef CONTROLLER_MODULE_H
#define CONTROLLER_MODULE_H
#include "Arduino.h"
#include <functional>

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "../DroneLinkMsg.h"
#include "I2CBaseModule.h"
#include "LinkedList.h"

#include "SSD1306Wire.h"

/*
@I2CAddress        0x3c
@default addr = 60
*/ 
#define CONTROLLER_OLED_I2C_ADDRESS  0x3c

// -----------------------------------------------------------------------------------
// pubs
// -----------------------------------------------------------------------------------
#define CONTROLLER_PARAM_ENTRIES     (I2CBASE_PARAM_ENTRIES + 0)


// -----------------------------------------------------------------------------------
// subs
// -----------------------------------------------------------------------------------
// @sub 20;21;f;1;select;Select button (e.g. JoyStick right button)
#define CONTROLLER_SUB_SELECT              (I2CBASE_SUBCLASS_PARAM_START+10) //20
#define CONTROLLER_SUB_SELECT_ADDR         (I2CBASE_SUBCLASS_PARAM_START+11) //21
#define CONTROLLER_SUB_SELECT_E            0

// @sub 22;23;f;1;cancel;Cancel button (e.g. JoyStick left button)
#define CONTROLLER_SUB_CANCEL              (I2CBASE_SUBCLASS_PARAM_START+12) //22
#define CONTROLLER_SUB_CANCEL_ADDR         (I2CBASE_SUBCLASS_PARAM_START+13) //23
#define CONTROLLER_SUB_CANCEL_E            1

// @sub 24;25;f;1;yAxis;yAxis for controlling menus (e.g. JoyStick right yAxis)
#define CONTROLLER_SUB_YAXIS               (I2CBASE_SUBCLASS_PARAM_START+14) //24
#define CONTROLLER_SUB_YAXIS_ADDR          (I2CBASE_SUBCLASS_PARAM_START+15) //25
#define CONTROLLER_SUB_YAXIS_E             2

// @sub 26;27;f;1;arm;Arm switch 
#define CONTROLLER_SUB_ARM                 (I2CBASE_SUBCLASS_PARAM_START+14) //26
#define CONTROLLER_SUB_ARM_ADDR            (I2CBASE_SUBCLASS_PARAM_START+15) //27
#define CONTROLLER_SUB_ARM_E               3

#define CONTROLLER_SUBS                4

// strings
static const char CONTROLLER_STR_CONTROLLER[] PROGMEM = "Controller";


/*
-------------------------------------------------------------------------------------
*/

struct CONTROLLER_DISPLAY_INFO {
  char name[DRONE_LINK_MSG_MAX_PAYLOAD];
  uint32_t position[2];
  uint8_t precision;
  uint8_t page;
  DRONE_LINK_MSG value;
};

struct CONTROLLER_CONTROL_INFO {
  char name[DRONE_LINK_MSG_MAX_PAYLOAD];
  DRONE_LINK_MSG value;
  DRONE_LINK_ADDR target;
};


/*
-------------------------------------------------------------------------------------
*/

struct CONTROLLER_MENU_ITEM {
  uint8_t menu;
  uint8_t data;  // any menu data associated, e.g. node id
  void *dataPointer; // use carefully!
  String name;
};


struct CONTROLLER_MENU_STATE {
  //uint8_t id;  // menu id - implicit
  uint8_t selected; // which item was selected
  uint8_t backTo;  // which menu item to go to if the left button is pushed
  String name;  // menu name
  IvanLinkedList::LinkedList<CONTROLLER_MENU_ITEM> items;
  //DrawMenuItemHandler drawMenuItemHandler;
  //SelectMenuHandler selectHandler;
};

#define CONTROLLER_MENU_ROOT       0

#define CONTROLLER_MENU_COUNT      1

#define CONTROLLER_MENU_INCREMENT_DATA_VALUE   255




/*
-------------------------------------------------------------------------------------
*/

// dhysteresis array indices
#define CONTROLLER_HYS_SELECT       0
#define CONTROLLER_HYS_CANCEL       1

// class
class ControllerModule:  public I2CBaseModule {
protected:

  char _title[DRONE_LINK_MSG_MAX_PAYLOAD];

  IvanLinkedList::LinkedList<CONTROLLER_DISPLAY_INFO*> _displayItems;
  IvanLinkedList::LinkedList<CONTROLLER_CONTROL_INFO*> _controlItems;

  uint8_t _brightness;

  boolean _armed;  // true if we are sending control data

  int _scroll;  // index of first item shown on screen i.e. top
  float _spinner;

  boolean _inMenu;  // whether menu system is active

  uint8_t _page;
  uint8_t _maxPage;

  uint8_t _menu;  // active menu
  uint8_t _lastMenu;  // last menu drawn
  CONTROLLER_MENU_STATE _menus[CONTROLLER_MENU_COUNT];

  boolean _buttons[2];     // flag if ui button pressed and needs handling
  boolean _hysteresis[2];  // hysteresis flags for UI controls

  SSD1306Wire *_display;
  DroneLinkMsg _sendMsg;
  DroneLinkMsg _queryMsg;
public:

  ControllerModule(uint8_t id, DroneSystem* ds);
  ~ControllerModule();

  void bindSubscriptions();

  void parseAddress(DRONE_LINK_ADDR *addressInfo, char * address);

  void loadConfiguration(const char* filename);

  void setMenuItem(uint8_t menu, uint8_t item, String name, uint8_t data, void* dataPointer, uint8_t nextMenu);

  

  void drawMenu();

  void manageMenu();

  void manageBinding();  // if binding is active

  void clear();
  void drawSpinner();

  void doReset();

  void doShutdown();

  void handleLinkMessage(DroneLinkMsg *msg);

  void setup();

  void loop();


};

#endif
