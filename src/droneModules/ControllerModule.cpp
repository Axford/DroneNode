#include "ControllerModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../DroneModuleManager.h"
#include "OLEDTomThumbFont.h"

/*

Menu structure:

NB: will attempt to load last binding at startup

Root = info for active binding OR indicator that is unbound
  Load config
    * Select config to load (from SPIFFS)

  Start new config
    * List of nodes to create binding (from DroneLinkManager nodeInfo map)
    * Selecting a node establishes an empty binding and goes to edit binding menu

  Edit current binding
    * Shows the list of control axes and their current bindings - i.e. the 6 joystick axes
    * Pick an axes to edit or establish a binding
    Edit Axis menu
      Bind -> Select module
        * Shows a list of modules available on the target node (continuously discovered in background)
        Select parameter
          * Shows a list of parameters for the selected module (continusouly discovered in background)
          * Once selected, creates the binding and returns to the Edit Current Binding menu
      Remove binding (if already bound)

  Save current binding (only available if a binding is active)
    * Saves current binding (using node name as reference) and sets flag for last used config

*/




ControllerModule::ControllerModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  I2CBaseModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(CONTROLLER_STR_CONTROLLER));
   _addr = CONTROLLER_OLED_I2C_ADDRESS;

   // init query msg
   _queryMsg.source(_dlm->node());
   _queryMsg.type(DRONE_LINK_MSG_TYPE_NAMEQUERY);
   _queryMsg.length(1);
   _queryMsg._msg.payload.uint8[0] = _dlm->node();

   // init axes
   for (uint8_t i=0; i<8; i++) {
     _axes[i] = 0;
     _neutral[i] = true;
   }

   // binding
   _isBound = false;

   // configure menus
   _menu = CONTROLLER_MENU_ROOT;
   _scroll = 0;
   _syncMenusTimer = 0;

   // defaults
   for (uint8_t i=0; i<sizeof(CONTROLLER_MENUS); i++) {
     _menus[i].selected = 0;
   }

   _menus[CONTROLLER_MENU_ROOT].name = strdup(PSTR("Root"));
   _menus[CONTROLLER_MENU_ROOT].backTo = CONTROLLER_MENU_ROOT;
   setMenuItem(CONTROLLER_MENU_ROOT, 0, strdup(PSTR("Main")), CONTROLLER_MENU_MAIN);

   _menus[CONTROLLER_MENU_MAIN].name = strdup(PSTR("Main"));
   _menus[CONTROLLER_MENU_MAIN].backTo = CONTROLLER_MENU_ROOT;
   setMenuItem(CONTROLLER_MENU_MAIN, 0, strdup(PSTR("Start")), CONTROLLER_MENU_START);

   _menus[CONTROLLER_MENU_START].name = strdup(PSTR("Start: Select a node"));
   _menus[CONTROLLER_MENU_START].backTo = CONTROLLER_MENU_MAIN;

   _menus[CONTROLLER_MENU_CREATE].name = strdup(PSTR("Create binding"));
   _menus[CONTROLLER_MENU_CREATE].backTo = CONTROLLER_MENU_MAIN;


   // subs


   // outputs
   initParams(CONTROLLER_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[CONTROLLER_PARAM_LEFT_E];
   param->param = CONTROLLER_PARAM_LEFT;
   setParamName(FPSTR(DRONE_STR_LEFT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   param->data.uint8[0] = 4;

   param = &_params[CONTROLLER_PARAM_RIGHT_E];
   param->param = CONTROLLER_PARAM_RIGHT;
   setParamName(FPSTR(DRONE_STR_RIGHT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   param->data.uint8[0] = 5;
}

ControllerModule::~ControllerModule() {
  if (_display) delete _display;
}


void ControllerModule::doReset() {
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_bus);

  _display->init();
  //_display->resetDisplay();

  _display->flipScreenVertically();
  _display->setFont(ArialMT_Plain_10);
}


void ControllerModule::doShutdown() {
  DroneModule::doShutdown(); // disables module

  DroneWire::selectChannel(_bus);

  // write shutdown message to screen
  // clear the display
  _display->clear();

  _display->setColor(WHITE);
  _display->setFont(ArialMT_Plain_10);

  _display->setTextAlignment(TEXT_ALIGN_LEFT);
  _display->drawString(0, 30, F("Restarting..."));

  // write the buffer to the display
  _display->display();
}


void ControllerModule::handleLinkMessage(DroneLinkMsg *msg) {
  // intercept values for joysticks

  // left
  if (msg->channel() == _params[CONTROLLER_PARAM_LEFT_E].data.uint8[0]) {
    if (msg->param() >= 8 && msg->param() <= 11) {
      if (msg->type() == DRONE_LINK_MSG_TYPE_FLOAT) {
        _axes[ CONTROLLER_AXIS_LEFT_X + msg->param() - 8] = msg->_msg.payload.f[0];
      }
    }
  }

  // right
  if (msg->channel() == _params[CONTROLLER_PARAM_RIGHT_E].data.uint8[0]) {
    if (msg->param() >= 8 && msg->param() <= 11) {
      if (msg->type() == DRONE_LINK_MSG_TYPE_FLOAT) {
        _axes[ CONTROLLER_AXIS_RIGHT_X + msg->param() - 8] = msg->_msg.payload.f[0];
      }
    }
  }

  DroneModule::handleLinkMessage(msg);
}


void ControllerModule::loadConfiguration(JsonObject &obj) {
  I2CBaseModule::loadConfiguration(obj);

  // instantiate sensor object, now _addr is known
  _display = new SSD1306Wire(_addr, SDA, SCL);

  // read joystick channels
  _params[CONTROLLER_PARAM_LEFT_E].data.uint8[0] = obj[DRONE_STR_LEFT] | _params[CONTROLLER_PARAM_LEFT_E].data.uint8[0];
  _params[CONTROLLER_PARAM_RIGHT_E].data.uint8[0] = obj[DRONE_STR_RIGHT] | _params[CONTROLLER_PARAM_RIGHT_E].data.uint8[0];

  // make sure we're subscribed to the joystick channels
  _dlm->subscribe(_params[CONTROLLER_PARAM_LEFT_E].data.uint8[0], this, DRONE_LINK_PARAM_ALL);
  _dlm->subscribe(_params[CONTROLLER_PARAM_RIGHT_E].data.uint8[0], this, DRONE_LINK_PARAM_ALL);
}


void ControllerModule::setMenuItem(uint8_t menu, uint8_t item, char* name, uint8_t nextMenu) {
  CONTROLLER_MENU_ITEM tempItem;
  strcpy(tempItem.name, name);
  tempItem.menu = nextMenu;

  if (item >= _menus[menu].items.size()) {
    // push
    _menus[menu].items.add(tempItem);
  } else {
    // update
    _menus[menu].items.set(item, tempItem);
  }
}


void ControllerModule::manageRoot(boolean syncMenu) {
  if (_isBound) {
    _display->drawString(0, 12, "Bound");

  } else {
    _display->drawString(0, 12, "Unbound - click to start");
  }
}

void ControllerModule::manageMain(boolean syncMenu) {
  drawMenu();
}

void ControllerModule::manageStart(boolean syncMenu) {
  if (syncMenu) {
    Serial.println("Syncing start menu");
    uint8_t j=0; // menu index
    char temp[16];
    DRONE_LINK_NODE_INFO* nodeInfo;
    for (uint8_t i= _dlm->minPeer(); i<=_dlm->maxPeer(); i++) {
      nodeInfo = _dlm->getNodeInfo(i);
      if (nodeInfo != NULL) {
        if (nodeInfo->name != NULL) {
          setMenuItem(CONTROLLER_MENU_START, j, nodeInfo->name, CONTROLLER_MENU_CREATE);
        } else {
          itoa(i, temp, 10);
          setMenuItem(CONTROLLER_MENU_START, j, temp, CONTROLLER_MENU_CREATE);
        }

        j++;
      }
    }
    Serial.println("done");

  }

  drawMenu();
}

void ControllerModule::manageCreate(boolean syncMenu) {
  // create new binding

  _display->drawString(0, 12, "New binding created");
  _display->drawString(0, 24, "Click to configure axes");
}




void ControllerModule::drawMenu() {
  // draw menu items
  _display->setTextAlignment(TEXT_ALIGN_LEFT);
  for (uint8_t i=0; i< _menus[_menu].items.size(); i++) {
    if (i == _menus[_menu].selected) {
      _display->setColor(WHITE);
      _display->fillRect(0, 15 + i*12, 128, 11);
      _display->setColor(BLACK);
    } else {
      _display->setColor(WHITE);
    }

    if (_menus[_menu].items[i].name) {
      _display->drawString(2, 14 + i*12, _menus[_menu].items[i].name);
    } else {
      _display->drawString(2, 14 + i*12, String(i) + " ?");
    }

  }
}


void ControllerModule::loop() {
  I2CBaseModule::loop();

  unsigned long loopTime = millis();

  boolean syncMenu = false;
  if (loopTime > _syncMenusTimer + 2000) {
    syncMenu = true;
    _syncMenusTimer = loopTime;
  }

  if ((_axes[CONTROLLER_AXIS_RIGHT_B]) < 0) _neutral[CONTROLLER_AXIS_RIGHT_B] = true;
  if ((_axes[CONTROLLER_AXIS_LEFT_B]) < 0) _neutral[CONTROLLER_AXIS_LEFT_B] = true;
  if (abs(_axes[CONTROLLER_AXIS_RIGHT_Y]) <0.2) _neutral[CONTROLLER_AXIS_RIGHT_Y] = true;


  // buttons - positive is pressed
  if (_axes[CONTROLLER_AXIS_RIGHT_B] > 0 &&  _neutral[CONTROLLER_AXIS_RIGHT_B]) {
    // right button pressed
    _menu = (CONTROLLER_MENUS)_menus[_menu].items[_menus[_menu].selected].menu;
    _neutral[CONTROLLER_AXIS_RIGHT_B] = false;
  }

  if (_axes[CONTROLLER_AXIS_LEFT_B] > 0 &&  _neutral[CONTROLLER_AXIS_LEFT_B]) {
    // left button pressed
    // go back
    _menu = (CONTROLLER_MENUS)_menus[_menu].backTo;
    _neutral[CONTROLLER_AXIS_LEFT_B] = false;
  }

  if (_axes[CONTROLLER_AXIS_RIGHT_Y] > 0.5 &&  _neutral[CONTROLLER_AXIS_RIGHT_Y]) {
    // right up
    if (_menus[_menu].selected> 0)
      _menus[_menu].selected--;
    if (_menus[_menu].selected < _scroll ) _scroll = _menus[_menu].selected;
    _neutral[CONTROLLER_AXIS_RIGHT_Y] = false;
  }

  if (_axes[CONTROLLER_AXIS_RIGHT_Y] < -0.5 &&  _neutral[CONTROLLER_AXIS_RIGHT_Y]) {
    // right down
    if (_menus[_menu].selected < _menus[_menu].items.size()-1)
      _menus[_menu].selected++;
    if (_menus[_menu].selected > _scroll + 4) _scroll++;
    _neutral[CONTROLLER_AXIS_RIGHT_Y] = false;
  }
  _scroll = max(min((int)(_menus[_menu].items.size() - 4), (int)_scroll), 0);


  DroneWire::selectChannel(_bus);

  // clear the display
  _display->clear();

  _display->setColor(WHITE);
  _display->fillRect(0,0,128,12);

  _display->setColor(BLACK);
  _display->setFont(ArialMT_Plain_10);

  // menu title
  _display->setTextAlignment(TEXT_ALIGN_LEFT);
  _display->drawString(2, 0, String(_menu) + "." + _menus[_menu].name);

  // draw menu size for debugging
  _display->setTextAlignment(TEXT_ALIGN_RIGHT);
  _display->setColor(BLACK);
  _display->setFont(TomThumb4x6);
  _display->drawString(128, 1, String(_menus[_menu].items.size()));

  // draw active menu
  _display->setColor(WHITE);
  _display->setFont(ArialMT_Plain_10);

  switch(_menu) {
    case CONTROLLER_MENU_ROOT: manageRoot(syncMenu); break;
    case CONTROLLER_MENU_MAIN: manageMain(syncMenu); break;
    case CONTROLLER_MENU_START: manageStart(syncMenu); break;
    case CONTROLLER_MENU_CREATE: manageCreate(syncMenu); break;
  }

  // write the buffer to the display
  _display->display();

}
