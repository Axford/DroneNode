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

#define CONTROLLER_ARM_BUTTON  PIN_OUT0_0

// pubs


#define CONTROLLER_PARAM_LEFT        (I2CBASE_SUBCLASS_PARAM_START+0)  //10 - channel for left joystick
#define CONTROLLER_PARAM_LEFT_E      (I2CBASE_PARAM_ENTRIES+0)

#define CONTROLLER_PARAM_RIGHT       (I2CBASE_SUBCLASS_PARAM_START+1)   // channel for right joystick
#define CONTROLLER_PARAM_RIGHT_E     (I2CBASE_PARAM_ENTRIES+1)

#define CONTROLLER_PARAM_TELEMETRY   (I2CBASE_SUBCLASS_PARAM_START+2)   // channel for telemetry module
#define CONTROLLER_PARAM_TELEMETRY_E (I2CBASE_PARAM_ENTRIES+2)

#define CONTROLLER_PARAM_POWER       (I2CBASE_SUBCLASS_PARAM_START+3)   // channel for INA219 module
#define CONTROLLER_PARAM_POWER_E     (I2CBASE_PARAM_ENTRIES+3)

#define CONTROLLER_PARAM_ENTRIES     (I2CBASE_PARAM_ENTRIES + 4)


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

#define CONTROLLER_DISCOVERY_INTERVAL   250


//typedef ustd::function<void(void*, boolean)> ManageMenuHandler;

//typedef std::function<void(void*, uint8_t index, uint8_t y)> DrawMenuItemHandler;

struct CONTROLLER_PARAM_INFO {
  uint8_t param;
  char name[17];
  uint8_t paramTypeLength;
};

struct CONTROLLER_CHANNEL_INFO {
  uint8_t channel;
  char name[17];
  uint8_t numFloats;
  IvanLinkedList::LinkedList<CONTROLLER_PARAM_INFO*> *params;
};

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
#define CONTROLLER_MENU_MAIN       1
#define CONTROLLER_MENU_START      2
#define CONTROLLER_MENU_CREATE     3
#define CONTROLLER_MENU_EDIT       4
#define CONTROLLER_MENU_EDITAXIS   5
#define CONTROLLER_MENU_INVERTAXIS 6
#define CONTROLLER_MENU_CLEARAXIS  7
#define CONTROLLER_MENU_BINDAXIS   8  // select module
#define CONTROLLER_MENU_BINDAXIS2  9  // select parameter
#define CONTROLLER_MENU_BINDAXIS3  10  // complete binding
#define CONTROLLER_MENU_CLEAR      11
#define CONTROLLER_MENU_EDITINFO   12

#define CONTROLLER_MENU_COUNT      13

#define CONTROLLER_MENU_INCREMENT_DATA_VALUE   255

//static_assert(CONTROLLER_MENU_COUNT == 5, "Incorrect menu size");

// axis indices
#define CONTROLLER_AXIS_LEFT_X   0
#define CONTROLLER_AXIS_LEFT_Y   1
#define CONTROLLER_AXIS_LEFT_Z   2
#define CONTROLLER_AXIS_LEFT_B   3
#define CONTROLLER_AXIS_RIGHT_X  4
#define CONTROLLER_AXIS_RIGHT_Y  5
#define CONTROLLER_AXIS_RIGHT_Z  6
#define CONTROLLER_AXIS_RIGHT_B  7

#define CONTROLLER_INFO_COUNT    4

#define LIPO_MIN_V    3.7f
#define LIPO_MAX_V    4.2f

// class
class ControllerModule:  public I2CBaseModule {
protected:
  float _axes[8];
  boolean _invert[8];  // axis inversion
  boolean _neutral[8];  // set true if entered neutral deadband, do provide hysterisis for menus

  DRONE_LINK_ADDR _bindings[8];
  String _bindingLabels[8]; // friendly binding labels (i.e. named channel > param)

  DroneLinkMsg _info[CONTROLLER_INFO_COUNT];
  String _infoLabels[CONTROLLER_INFO_COUNT]; // friendly info labels (i.e. named channel > param)

  float _RSSI;  // last received RSSI from telemetry module

  uint8_t _brightness;
  unsigned long _syncMenusTimer;

  boolean _bindingAxis;  // true if we're in the menu for binding an axis, false if we're binding an info item

  boolean _isBound;
  String _bindingName;
  uint8_t _binding;  // node id we're bound to

  uint8_t _menu;  // active menu
  uint8_t _lastMenu;  // last menu drawn

  float _spinner;
  float _cellVoltage;  // cell battery voltage measured from INA219
  float _batteryCapacity; // 0..1 as approx battery %

  boolean _armed;  // true if we are sending control data and overriding local navigation

  // track modules for binding
  boolean _channelInfoChanged;
  unsigned long _lastDiscovery;
  IvanLinkedList::LinkedList<CONTROLLER_CHANNEL_INFO*> _availChannels;

  CONTROLLER_MENU_STATE _menus[CONTROLLER_MENU_COUNT];

  int _scroll;  // index of first item shown on screen i.e. top

  SSD1306Wire *_display;
  DroneLinkMsg _queryMsg;
  DroneLinkMsg _sendMsg;
public:

  ControllerModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);
  ~ControllerModule();

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void clear();

  void doReset();

  void doShutdown();

  void arm();
  void disarm();

  void handleLinkMessage(DroneLinkMsg *msg);

  //void loadConfiguration(JsonObject &obj);

  void setup();

  CONTROLLER_PARAM_INFO* getParamInfo(CONTROLLER_CHANNEL_INFO *channel, uint8_t param);
  CONTROLLER_CHANNEL_INFO* getChannelInfo(uint8_t channel);

  void setMenuItem(uint8_t menu, uint8_t item, String name, uint8_t data, void* dataPointer, uint8_t nextMenu);

  void publishEntry(uint8_t i);

  void manageRoot(boolean syncMenu);
  void manageStart(boolean syncMenu);
  void manageClear(boolean syncMenu);

  void manageCreate(boolean syncMenu);

  void manageEdit(boolean syncMenu);
  void drawEditMenuItem(uint8_t index, uint8_t y);

  void manageEditAxis(boolean syncMenu);

  void manageBindAxis(boolean syncMenu);
  void drawBindAxisMenuItem(uint8_t index, uint8_t y);

  void manageBindAxis2(boolean syncMenu);
  void manageBindAxis3(boolean syncMenu);

  void manageEditInfo(boolean syncMenu);
  void drawEditInfoMenuItem(uint8_t index, uint8_t y);

  void drawMenu();

  void drawSpinner();

  void loop();


};

#endif
