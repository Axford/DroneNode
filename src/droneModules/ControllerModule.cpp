#include "ControllerModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../DroneModuleManager.h"
#include "OLEDTomThumbFont.h"


/*
TODO
 - Save binding config
 - Load binding config
 - Create info bindings from all params
*/

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


  Edit info map
    * Once a binding is completed


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
   _queryMsg._msg.payload.uint8[0] = 0;

   _sendMsg.source(_dlm->node());
   _sendMsg.type(DRONE_LINK_MSG_TYPE_FLOAT);
   _sendMsg.writable(true);
   _sendMsg.length(4);
   _sendMsg._msg.payload.f[0] = 0;

   _brightness = 0;
   _spinner = 0;

   _RSSI = 0;

   // configure menus
   _menu = CONTROLLER_MENU_ROOT;
   _scroll = 0;
   _syncMenusTimer = 0;
   _lastDiscovery = 0;

   // defaults
   for (uint8_t i=0; i<CONTROLLER_MENU_COUNT; i++) {
     _menus[i].selected = 0;
   }

   clear();

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

   param = &_params[CONTROLLER_PARAM_TELEMETRY_E];
   param->param = CONTROLLER_PARAM_TELEMETRY;
   setParamName(FPSTR(DRONE_STR_TELEMETRY), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   param->data.uint8[0] = 6;
}

ControllerModule::~ControllerModule() {
  if (_display) delete _display;
}


void ControllerModule::clear() {
  Log.noticeln(F("Clearing..."));
  _isBound = false;
  _bindingName = "";
  _channelInfoChanged = false;

  // init axes
  for (uint8_t i=0; i<8; i++) {
    _axes[i] = 0;
    _neutral[i] = true;
    _bindings[i].node = 255;
    _bindings[i].channel = 255;
    _bindings[i].param = 255;
    _bindingLabels[i] = "";
  }

  for (uint8_t i=0; i<CONTROLLER_INFO_COUNT; i++) {
    _info[i].node(255);
    _info[i].channel(255);
    _info[i].param(255);
    _info[i].length(1);
    _info[i].type(DRONE_LINK_MSG_TYPE_CHAR);
    _info[i].setUint8_t('?');
    _infoLabels[i] = "";
  }

  // reset channel / param tree
  for (uint8_t i=0; i < _availChannels.size(); i++) {
    CONTROLLER_CHANNEL_INFO* chanInfo = _availChannels.get(i);

    for (uint8_t j=0; j < chanInfo->params->size(); j++) {
      // clear memory allocated for param
      free(chanInfo->params->get(j));
    }
    chanInfo->params->clear();
    // delete params list

    // clear memory associaed with chanInfo
    free(chanInfo);
  }
  _availChannels.clear();
  Log.noticeln(F("done"));
}


void ControllerModule::doReset() {
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_bus);

  //_display->init();
  if (_display) _display->resetDisplay();

  //_display->flipScreenVertically();
}


void ControllerModule::doShutdown() {
  DroneModule::doShutdown(); // disables module

  DroneWire::selectChannel(_bus);

  // write shutdown message to screen
  // clear the display
  if (_display) {

    _display->clear();

    _display->setColor(WHITE);
    _display->setFont(ArialMT_Plain_10);

    _display->setTextAlignment(TEXT_ALIGN_CENTER);
    _display->drawString(64, 25, F("Restarting..."));

    // write the buffer to the display
    _display->display();
  }
}

CONTROLLER_PARAM_INFO* ControllerModule::getParamInfo(CONTROLLER_CHANNEL_INFO *channel, uint8_t param) {
  // find param
  for (uint8_t i = 0; i<channel->params->size(); i++) {
    if (channel->params->get(i)->param == param) {
      return channel->params->get(i);
    }
  }
  return NULL;
}


CONTROLLER_CHANNEL_INFO* ControllerModule::getChannelInfo(uint8_t channel) {
  // find module
  for (uint8_t i = 0; i<_availChannels.size(); i++) {
    if (_availChannels[i]->channel == channel) {
      return _availChannels[i];
    }
  }
  return NULL;
}

void ControllerModule::handleLinkMessage(DroneLinkMsg *msg) {
  // intercept values for joysticks

  // left
  uint8_t axis = 255;
  if (msg->channel() == _params[CONTROLLER_PARAM_LEFT_E].data.uint8[0]) {
    if (msg->param() >= 8 && msg->param() <= 11) {
      if (msg->type() == DRONE_LINK_MSG_TYPE_FLOAT) {
        axis = CONTROLLER_AXIS_LEFT_X + msg->param() - 8;
        _axes[ axis] = msg->_msg.payload.f[0];
      }
    }
  }

  // right
  if (msg->channel() == _params[CONTROLLER_PARAM_RIGHT_E].data.uint8[0]) {
    if (msg->param() >= 8 && msg->param() <= 11) {
      if (msg->type() == DRONE_LINK_MSG_TYPE_FLOAT) {
        axis = CONTROLLER_AXIS_RIGHT_X + msg->param() - 8;
        _axes[ axis ] = msg->_msg.payload.f[0];
      }
    }
  }

  // intercept RSSI for telemetry
  if (msg->channel() == _params[CONTROLLER_PARAM_TELEMETRY_E].data.uint8[0] &&
      msg->param() == 8 &&
      msg->type() == DRONE_LINK_MSG_TYPE_FLOAT) {
    _RSSI = msg->_msg.payload.f[0];
  }

  if (!_isBound) {
    _spinner += PI / 16.0;
  }

  // listen to channels / params for active binding
  // insert sort into relevant menu list
  if (_isBound && msg->node() == _binding) {
    _spinner += PI / 8.0;
    //Serial.println("new binding info");

    CONTROLLER_CHANNEL_INFO *chan = getChannelInfo(msg->channel());

    // intercept module names
    if (chan != NULL &&
        msg->type() == DRONE_LINK_MSG_TYPE_CHAR &&
        msg->param() == DRONE_MODULE_PARAM_NAME) {
          Log.noticeln("Received module name %u", msg->channel());
          memcpy(chan->name, msg->_msg.payload.c, msg->length());
          chan->name[msg->length()] = '\0';
          _channelInfoChanged = true;
        }

    // intercept param names
    if (chan != NULL &&
        msg->type() == DRONE_LINK_MSG_TYPE_NAME) {
          // lets see if this is a param we're interested in:
          CONTROLLER_PARAM_INFO *paramInfo = getParamInfo(chan, msg->param());
          if (paramInfo != NULL) {
            Log.noticeln("Received param name %u", msg->param());
            memcpy(paramInfo->name, msg->_msg.payload.c, msg->length());
            paramInfo->name[msg->length()] = '\0';
            _channelInfoChanged = true;
          }
        }

    // intercept info bindings
    for (uint8_t i=0; i<CONTROLLER_INFO_COUNT; i++) {
      if (msg->channel() == _info[i].channel() &&
          msg->param() == _info[i].param() &&
          msg->type() <= DRONE_LINK_MSG_TYPE_CHAR) {
        _info[i]._msg.paramTypeLength = msg->_msg.paramTypeLength;
        uint8_t len = msg->length();
        memcpy(_info[i]._msg.payload.c, msg->_msg.payload.c, len);
      }
    }


    // capture all displayable parameters
    if (msg->type() <= DRONE_LINK_MSG_TYPE_CHAR) {

      Log.noticeln("Received param info");

      // add to list
      if (chan == NULL) {
        Serial.print("new writable param: ");
        Serial.print(msg->channel());
        Serial.print(".");
        Serial.println(msg->param());

        chan = (CONTROLLER_CHANNEL_INFO*)malloc(sizeof(CONTROLLER_CHANNEL_INFO));
        chan->channel = msg->channel();
        chan->name[0] = '?';
        chan->name[1] = '\0';
        chan->numFloats = 0;
        chan->params = new IvanLinkedList::LinkedList<CONTROLLER_PARAM_INFO*>;

        Serial.print("linked list size: ");
        Serial.println(chan->params->size());

        _channelInfoChanged = true;

        _availChannels.add(chan);

        // sort
        _availChannels.sort([](CONTROLLER_CHANNEL_INFO *&a, CONTROLLER_CHANNEL_INFO *&b) {
          return a->channel - b->channel;
        });

        // fire off a channel name query
        /*
        _queryMsg.type(DRONE_LINK_MSG_TYPE_QUERY);
        _queryMsg.node(_binding);
        _queryMsg.channel(msg->channel());
        _queryMsg.param(DRONE_MODULE_PARAM_NAME);
        _dlm->publish(_queryMsg);
        */
      }

      // assuming chan is now available
      if (chan != NULL) {
        // store the param in chan info
        Serial.println("Storing param info");

        CONTROLLER_PARAM_INFO *paramInfo = getParamInfo(chan, msg->param());

        if (paramInfo == NULL) {
          Serial.print("new param: ");

          paramInfo = (CONTROLLER_PARAM_INFO*)malloc(sizeof(CONTROLLER_PARAM_INFO));
          paramInfo->param = msg->param();
          paramInfo->paramTypeLength = msg->_msg.paramTypeLength;
          paramInfo->name[0] = '?';
          paramInfo->name[1] = '\0';

          if (msg->type() == DRONE_LINK_MSG_TYPE_FLOAT &&
              msg->length() == 4 &&
              msg->writable()) {
            // only look for single value floats
            chan->numFloats++;
          }

          _channelInfoChanged = true;

          chan->params->add(paramInfo);

          // sort
          chan->params->sort([](CONTROLLER_PARAM_INFO *&a, CONTROLLER_PARAM_INFO *&b) {
            return a->param - b->param;
          });

          // fire off a param name query
          /*
          _queryMsg.type(DRONE_LINK_MSG_TYPE_NAMEQUERY);
          _queryMsg.node(_binding);
          _queryMsg.channel(msg->channel());
          _queryMsg.param(msg->param());
          _dlm->publish(_queryMsg);
          */
        }
      }
    }
  }

  DroneModule::handleLinkMessage(msg);
}


void ControllerModule::loadConfiguration(JsonObject &obj) {
  I2CBaseModule::loadConfiguration(obj);

  // instantiate sensor object, now _addr is known
  DroneWire::selectChannel(_bus);

  _display = new SSD1306Wire(_addr, SDA, SCL);

  // read joystick channels
  _params[CONTROLLER_PARAM_LEFT_E].data.uint8[0] = obj[DRONE_STR_LEFT] | _params[CONTROLLER_PARAM_LEFT_E].data.uint8[0];
  _params[CONTROLLER_PARAM_RIGHT_E].data.uint8[0] = obj[DRONE_STR_RIGHT] | _params[CONTROLLER_PARAM_RIGHT_E].data.uint8[0];

  // make sure we're subscribed to the joystick channels
  _dlm->subscribe(_params[CONTROLLER_PARAM_LEFT_E].data.uint8[0], this, DRONE_LINK_PARAM_ALL);
  _dlm->subscribe(_params[CONTROLLER_PARAM_RIGHT_E].data.uint8[0], this, DRONE_LINK_PARAM_ALL);
}


void ControllerModule::setup() {
  DroneModule::setup();

 _menus[CONTROLLER_MENU_ROOT].name = (F("Root"));
 _menus[CONTROLLER_MENU_ROOT].backTo = CONTROLLER_MENU_ROOT;
 setMenuItem(CONTROLLER_MENU_ROOT, 0, (F("Main")), 0, NULL, CONTROLLER_MENU_MAIN);

 _menus[CONTROLLER_MENU_MAIN].name = (F("Main"));
 _menus[CONTROLLER_MENU_MAIN].backTo = CONTROLLER_MENU_ROOT;
 setMenuItem(CONTROLLER_MENU_MAIN, 0, F("Start"), 0, NULL, CONTROLLER_MENU_START);
 setMenuItem(CONTROLLER_MENU_MAIN, 1, F("Edit Bindings"), 1, NULL, CONTROLLER_MENU_EDIT);
 setMenuItem(CONTROLLER_MENU_MAIN, 2, F("Edit Info"), 2, NULL, CONTROLLER_MENU_EDITINFO);
 setMenuItem(CONTROLLER_MENU_MAIN, 3, F("Clear"), 3, NULL, CONTROLLER_MENU_CLEAR);

 _menus[CONTROLLER_MENU_START].name = (F("Select a node"));
 _menus[CONTROLLER_MENU_START].backTo = CONTROLLER_MENU_MAIN;

 _menus[CONTROLLER_MENU_CREATE].name = (F("Create binding"));
 _menus[CONTROLLER_MENU_CREATE].backTo = CONTROLLER_MENU_MAIN;
 // dummy menu entry for navigation
 setMenuItem(CONTROLLER_MENU_CREATE, 0, (F("Edit")), 0, NULL, CONTROLLER_MENU_EDIT);

 _menus[CONTROLLER_MENU_EDIT].name = (F("Select axis to bind"));
 _menus[CONTROLLER_MENU_EDIT].backTo = CONTROLLER_MENU_MAIN;
 setMenuItem(CONTROLLER_MENU_EDIT, 0, F("LX"), 0, NULL, CONTROLLER_MENU_BINDAXIS);
 setMenuItem(CONTROLLER_MENU_EDIT, 1, (F("LY")), 1, NULL, CONTROLLER_MENU_BINDAXIS);
 setMenuItem(CONTROLLER_MENU_EDIT, 2, (F("LZ")), 2, NULL, CONTROLLER_MENU_BINDAXIS);
 setMenuItem(CONTROLLER_MENU_EDIT, 3, (F("RX")), 4, NULL, CONTROLLER_MENU_BINDAXIS);
 setMenuItem(CONTROLLER_MENU_EDIT, 4, (F("RY")), 5, NULL, CONTROLLER_MENU_BINDAXIS);
 setMenuItem(CONTROLLER_MENU_EDIT, 5, (F("RZ")), 6, NULL, CONTROLLER_MENU_BINDAXIS);

 _menus[CONTROLLER_MENU_EDITINFO].name = (F("Select item to bind"));
 _menus[CONTROLLER_MENU_EDITINFO].backTo = CONTROLLER_MENU_MAIN;
 for (uint8_t i=0; i<CONTROLLER_INFO_COUNT; i++)
  setMenuItem(CONTROLLER_MENU_EDITINFO, i, "Item "+String(i), i, NULL, CONTROLLER_MENU_BINDAXIS);


 _menus[CONTROLLER_MENU_BINDAXIS].name = (F("Select module"));
 _menus[CONTROLLER_MENU_BINDAXIS].backTo = CONTROLLER_MENU_EDIT;

 _menus[CONTROLLER_MENU_BINDAXIS2].name = (F("Select param"));
 _menus[CONTROLLER_MENU_BINDAXIS2].backTo = CONTROLLER_MENU_BINDAXIS;

 _menus[CONTROLLER_MENU_BINDAXIS3].name = (F("Bind complete"));
 _menus[CONTROLLER_MENU_BINDAXIS3].backTo = CONTROLLER_MENU_EDIT;

 _menus[CONTROLLER_MENU_CLEAR].name = (F("Binding Cleared"));
 _menus[CONTROLLER_MENU_CLEAR].backTo = CONTROLLER_MENU_MAIN;

 // Init display
 if (_display) {
   DroneWire::selectChannel(_bus);

   if (!_display->init()) {
     Log.errorln(F("display->init()"));
   }
   //_display->resetDisplay();

   _display->flipScreenVertically();

   _display->clear();
   _display->display();
   _display->setBrightness(_brightness);
 } else {
   //Serial.println("Err: _display not created");
 }
}


void ControllerModule::setMenuItem(uint8_t menu, uint8_t item, String name, uint8_t data, void* dataPointer, uint8_t nextMenu) {
  CONTROLLER_MENU_ITEM tempItem;
  //Serial.printf("Adding to menu %u, item %u \n", menu, item);

  tempItem.name = name;
  tempItem.menu = nextMenu;
  tempItem.data = data;
  tempItem.dataPointer = dataPointer;

  if (item >= _menus[menu].items.size()) {
    // push
    _menus[menu].items.add(tempItem);
  } else {
    // update
    _menus[menu].items.set(item, tempItem);
  }
}


void ControllerModule::manageRoot(boolean syncMenu) {
  _display->setColor(WHITE);

  if (_isBound) {
    // show any configured info bindings

    _display->setTextAlignment(TEXT_ALIGN_LEFT);
    uint8_t y;
    String valueStr = "";
    for (uint8_t i=0; i< CONTROLLER_INFO_COUNT; i++) {
      y = 14 + (i) *12;

      if (_info[i].param() < 255) {
        // draw label
        _display->setFont(TomThumb4x6);
        _display->setTextAlignment(TEXT_ALIGN_RIGHT);
        _display->drawString(58, y+4, _infoLabels[i]);

        // draw value
        uint8_t byteLen = (_info[i]._msg.paramTypeLength & 0xF)+1;
        uint8_t msgType = (_info[i]._msg.paramTypeLength >> 4) & 0x07;
        uint8_t numValues = (msgType == DRONE_LINK_MSG_TYPE_CHAR) ? 1 : (byteLen / DRONE_LINK_MSG_TYPE_SIZES[msgType]);

        _display->setTextAlignment(TEXT_ALIGN_LEFT);
        if (numValues > 1) {
          _display->setFont(TomThumb4x6);
        } else
          _display->setFont(ArialMT_Plain_10);


        valueStr = "";
        for (uint8_t j=0; j<numValues; j++) {
          if (j > 0) valueStr += ' ';

          switch(msgType) {
            case DRONE_LINK_MSG_TYPE_UINT8_T:
              valueStr += String(_info[i]._msg.payload.uint8[j]);
              break;
            case DRONE_LINK_MSG_TYPE_UINT32_T:
              valueStr += String(_info[i]._msg.payload.uint32[j]);
              break;
            case DRONE_LINK_MSG_TYPE_FLOAT:
              valueStr += String(_info[i]._msg.payload.f[j]);
              break;
            case DRONE_LINK_MSG_TYPE_CHAR:
              valueStr += String(_info[i]._msg.payload.c);
              break;
          }
        }

        _display->drawString(62, y + (numValues > 1 ? 4 : 0), valueStr);


      }
    }

    drawSpinner();

  } else {
    _menus[CONTROLLER_MENU_ROOT].name = "No Binding";

    _display->setTextAlignment(TEXT_ALIGN_CENTER);
    _display->drawString(64, 25, F("Click to start"));
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
          setMenuItem(CONTROLLER_MENU_START, j, nodeInfo->name, i, NULL, CONTROLLER_MENU_CREATE);
        } else {
          itoa(i, temp, 10);
          setMenuItem(CONTROLLER_MENU_START, j, temp, i, NULL, CONTROLLER_MENU_CREATE);
        }

        j++;
      }
    }
    Serial.println("done");

  }

  drawMenu();
  drawSpinner();
}

void ControllerModule::manageClear(boolean syncMenu) {
  _display->setColor(WHITE);

  if (_isBound) {
    clear();

  } else {
    _menus[CONTROLLER_MENU_ROOT].name = "No Binding";

    _display->setTextAlignment(TEXT_ALIGN_CENTER);
    _display->drawString(64, 25, F("Binding cleared"));
  }
}

void ControllerModule::manageCreate(boolean syncMenu) {
  // create new binding

  if (!_isBound) {
    _isBound = true;
    _bindingName = _menus[CONTROLLER_MENU_START].items[_menus[CONTROLLER_MENU_START].selected].name;
    _binding = _menus[CONTROLLER_MENU_START].items[_menus[CONTROLLER_MENU_START].selected].data;

    _menus[CONTROLLER_MENU_ROOT].name = String(_binding) + ">" + _bindingName;

    // make sure we're subscribed to the bound node and all its channels/params
    Log.noticeln("Subscribing Controller to: %u", _binding);
    _dlm->subscribe(_binding, 0, this, 0);
  }

  _display->setColor(WHITE);
  _display->drawString(0, 12, F("New binding created"));
  _display->drawString(0, 24, F("Click to configure axes"));
}

void ControllerModule::manageEdit(boolean syncMenu) {
  if (_isBound) {
    // pick an axis to edit binding
    _bindingAxis = true;
    _menus[CONTROLLER_MENU_BINDAXIS].backTo = CONTROLLER_MENU_EDIT;
    drawMenu();
  } else {
    // redirect to start
    _menu = CONTROLLER_MENU_START;
  }
}

void ControllerModule::drawEditMenuItem(uint8_t index, uint8_t y) {
  if (_menus[_menu].items[index].name) {
    _display->setFont(ArialMT_Plain_10);
    _display->drawString(2, y, _menus[_menu].items[index].name);
  }


  uint8_t axis = _menus[_menu].items[index].data;
  if (_bindings[axis].param != 255) {
    _display->setFont(TomThumb4x6);
    _display->drawString(20, y+4, _bindingLabels[axis]);
  }
}


void ControllerModule::manageBindAxis(boolean syncMenu) {
  //Serial.println("Manage bindAxis");

  if (_channelInfoChanged) {
    //Serial.println("Syncing channel list");
    //Serial.println(_availChannels.size());
    CONTROLLER_CHANNEL_INFO* chanInfo;
    for (uint8_t i=0; i<_availChannels.size(); i++) {
      chanInfo = _availChannels[i];
      Serial.println(chanInfo->channel);

      if (chanInfo->name[0] == '?') {
        // send a fresh name query
        if (millis() > _lastDiscovery + CONTROLLER_DISCOVERY_INTERVAL) {
          //Serial.println("requery module name");
          _queryMsg.type(DRONE_LINK_MSG_TYPE_QUERY);
          _queryMsg.node(_binding);
          _queryMsg.channel(chanInfo->channel);
          _queryMsg.param(DRONE_MODULE_PARAM_NAME);
          _dlm->publish(_queryMsg);

          _lastDiscovery = millis();
        }
      }

      setMenuItem(CONTROLLER_MENU_BINDAXIS, i, chanInfo->name, chanInfo->channel, chanInfo, CONTROLLER_MENU_BINDAXIS2);
    }
    //Serial.println("done");

    _channelInfoChanged = false;
  }

  // clear the param submenu items
  _menus[CONTROLLER_MENU_BINDAXIS2].items.clear();

  drawMenu();
  drawSpinner();
}


void ControllerModule::drawBindAxisMenuItem(uint8_t index, uint8_t y) {
  uint8_t channel = _menus[_menu].items[index].data;
  String temp = String(channel) + ".";
  if (_menus[_menu].items[index].name) {
    temp += _menus[_menu].items[index].name;
  } else {
    temp += " ?";
  }
  // get number of params for this channel
  CONTROLLER_CHANNEL_INFO *chan = getChannelInfo(channel);

  if (chan != NULL) {
    temp += " (";
    if (_bindingAxis) temp += String(chan->numFloats) +'/';
    temp += String(chan->params->size()) + ")";
  }
  _display->drawString(2, y, temp);
}


void ControllerModule::manageBindAxis2(boolean syncMenu) {
  //Serial.println("Manage bindAxis");

  if (_channelInfoChanged || syncMenu) {
    //Serial.println("Syncing param list");

    // get channel reference
    CONTROLLER_CHANNEL_INFO* chanInfo = (CONTROLLER_CHANNEL_INFO*)_menus[CONTROLLER_MENU_BINDAXIS].items[ _menus[CONTROLLER_MENU_BINDAXIS].selected ].dataPointer;

    // for each param
    uint8_t j = 0;
    for (uint8_t i=0; i<chanInfo->params->size(); i++) {
      CONTROLLER_PARAM_INFO *paramInfo = chanInfo->params->get(i);

      if ( (paramInfo->paramTypeLength == (DRONE_LINK_MSG_WRITABLE | (DRONE_LINK_MSG_TYPE_FLOAT << 4) | 0x3)) ||
            !_bindingAxis) {
        if (paramInfo->name[0] == '?') {
          // send a fresh name query
          if (millis() > _lastDiscovery + CONTROLLER_DISCOVERY_INTERVAL) {
            //Serial.println("requery param name");
            _queryMsg.type(DRONE_LINK_MSG_TYPE_NAMEQUERY);
            _queryMsg.node(_binding);
            _queryMsg.channel(chanInfo->channel);
            _queryMsg.param(paramInfo->param);
            _dlm->publish(_queryMsg);

            _lastDiscovery = millis();
          }
        }

        setMenuItem(CONTROLLER_MENU_BINDAXIS2, j, paramInfo->name, paramInfo->param, paramInfo, CONTROLLER_MENU_BINDAXIS3);
        j++;
      }

    }
    //Serial.println("done");
    _channelInfoChanged = false;
  }

  drawMenu();
  drawSpinner();
}


void ControllerModule::manageBindAxis3(boolean syncMenu) {
  // complete new binding

  if (_bindingAxis) {
    // get axis
    uint8_t axis = _menus[CONTROLLER_MENU_EDIT].items[ _menus[CONTROLLER_MENU_EDIT].selected ].data;

    // get channel reference
    CONTROLLER_CHANNEL_INFO* chanInfo = (CONTROLLER_CHANNEL_INFO*)_menus[CONTROLLER_MENU_BINDAXIS].items[ _menus[CONTROLLER_MENU_BINDAXIS].selected ].dataPointer;

    // get param reference
    CONTROLLER_PARAM_INFO* paramInfo = (CONTROLLER_PARAM_INFO*)_menus[CONTROLLER_MENU_BINDAXIS2].items[ _menus[CONTROLLER_MENU_BINDAXIS2].selected ].dataPointer;

    _bindings[axis].node = _binding;
    _bindings[axis].channel = chanInfo->channel;
    _bindings[axis].param = paramInfo->param;

    _bindingLabels[axis] = String(chanInfo->name) + "." + String(paramInfo->name);

    // immediately redirect to edit menu
    _menu = CONTROLLER_MENU_EDIT;

  } else {
    // binding info

    // get info item
    uint8_t item = _menus[CONTROLLER_MENU_EDITINFO].items[ _menus[CONTROLLER_MENU_EDITINFO].selected ].data;

    // get channel reference
    CONTROLLER_CHANNEL_INFO* chanInfo = (CONTROLLER_CHANNEL_INFO*)_menus[CONTROLLER_MENU_BINDAXIS].items[ _menus[CONTROLLER_MENU_BINDAXIS].selected ].dataPointer;

    // get param reference
    CONTROLLER_PARAM_INFO* paramInfo = (CONTROLLER_PARAM_INFO*)_menus[CONTROLLER_MENU_BINDAXIS2].items[ _menus[CONTROLLER_MENU_BINDAXIS2].selected ].dataPointer;

    _info[item].node(_binding);
    _info[item].channel(chanInfo->channel);
    _info[item].param(paramInfo->param);

    _infoLabels[item] = String(chanInfo->name) + "." + String(paramInfo->name);

    // immediately redirect to edit menu
    _menu = CONTROLLER_MENU_EDITINFO;
  }


}



void ControllerModule::manageEditInfo(boolean syncMenu) {
  if (_isBound) {
    // pick an axis to edit binding
    _bindingAxis = false;
    _menus[CONTROLLER_MENU_BINDAXIS].backTo = CONTROLLER_MENU_EDITINFO;
    drawMenu();
  } else {
    // redirect to start
    _menu = CONTROLLER_MENU_START;
  }
}

void ControllerModule::drawEditInfoMenuItem(uint8_t index, uint8_t y) {

  _display->setFont(ArialMT_Plain_10);
  _display->drawString(2, y, String(index));

  uint8_t item = _menus[_menu].items[index].data;
  if (_info[item].param() != 255) {
    _display->setFont(TomThumb4x6);
    _display->drawString(20, y+4, _infoLabels[item]);
  }
}



void ControllerModule::drawMenu() {
  // draw menu items
  _display->setTextAlignment(TEXT_ALIGN_LEFT);
  uint8_t y;
  for (uint8_t i=0; i< _menus[_menu].items.size(); i++) {
    if (i >= _scroll) {
      y = 14 + (i - _scroll) *12;

      if (i == _menus[_menu].selected) {
        _display->setColor(WHITE);
        _display->fillRect(0, y+1, 128, 11);
        _display->setColor(BLACK);
      } else {
        _display->setColor(WHITE);
      }

      if (_menu == CONTROLLER_MENU_BINDAXIS) {
        drawBindAxisMenuItem(i,y);
      } else if (_menu == CONTROLLER_MENU_EDIT) {
        drawEditMenuItem(i,y);
      } else if (_menu == CONTROLLER_MENU_EDITINFO) {
        drawEditInfoMenuItem(i,y);
      } else {
        if (_menus[_menu].items[i].name) {
          _display->drawString(2, y, String(_menus[_menu].items[i].data) + "." + _menus[_menu].items[i].name);
        } else {
          _display->drawString(2, y, String(_menus[_menu].items[i].data) + ". ?");
        }
      }
    }
  }
}


void ControllerModule::drawSpinner() {
  int cx = 122;
  int cy = 58;

  int x = 4 * cos(_spinner);
  int y = 4 * sin(_spinner);

  _display->setColor(WHITE);

  _display->drawLine(cx - x, cy - y,  cx + x, cy + y);
}


void ControllerModule::loop() {
  I2CBaseModule::loop();

  //Serial.println("loop");

  DroneWire::selectChannel(_bus);


  // do we have an active binding to send to?
  // in which case keep spamming values to allow for any packet loss
  if (_isBound && _menu == CONTROLLER_MENU_ROOT) {
    _sendMsg.node(_binding);
    for (uint8_t i=0; i<8; i++) {
      if (_bindings[i].param <255) {
        _sendMsg.channel(_bindings[i].channel);
        _sendMsg.param(_bindings[i].param);
        _sendMsg._msg.payload.f[0] = _axes[i];
        _dlm->publish(_sendMsg);
      }
    }
  }


  unsigned long loopTime = millis();

  boolean syncMenu = false;
  if (loopTime > _syncMenusTimer + 1000) {
    syncMenu = true;
    _syncMenusTimer = loopTime;
  }
  if (_menu != _lastMenu) {
    syncMenu = true;
  }

  if ((_axes[CONTROLLER_AXIS_RIGHT_B]) < 0) _neutral[CONTROLLER_AXIS_RIGHT_B] = true;
  if ((_axes[CONTROLLER_AXIS_LEFT_B]) < 0) _neutral[CONTROLLER_AXIS_LEFT_B] = true;
  if (abs(_axes[CONTROLLER_AXIS_RIGHT_Y]) <0.2) _neutral[CONTROLLER_AXIS_RIGHT_Y] = true;


  // buttons - positive is pressed
  if (_axes[CONTROLLER_AXIS_RIGHT_B] > 0 &&  _neutral[CONTROLLER_AXIS_RIGHT_B]) {
    // right button pressed
    if (_menus[_menu].items.size() > 0) {
      _menu = _menus[_menu].items[_menus[_menu].selected].menu;
    }

    _neutral[CONTROLLER_AXIS_RIGHT_B] = false;
  }

  if (_axes[CONTROLLER_AXIS_LEFT_B] > 0 &&  _neutral[CONTROLLER_AXIS_LEFT_B]) {
    // left button pressed
    // go back
    _menu = _menus[_menu].backTo;
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
    if (_menus[_menu].selected > _scroll + 3) _scroll++;
    _neutral[CONTROLLER_AXIS_RIGHT_Y] = false;
  }

  // ensure selected is in range
  if (_menus[_menu].selected >= _menus[_menu].items.size())
    _menus[_menu].selected = _menus[_menu].items.size()-1;

  // ensure scroll in range when we change menus
  if (_menus[_menu].selected < _scroll) _scroll = _menus[_menu].selected;
  if (_menus[_menu].selected > _scroll + 3) _scroll = _menus[_menu].selected - 3;
  _scroll = max(min((int)(_menus[_menu].items.size() - 4), (int)_scroll), 0);

  // dim up on start
  if (_brightness < 254) _brightness+= 2;
  _display->setBrightness(_brightness);

  // clear the display
  _display->clear();

  _display->setColor(WHITE);
  _display->fillRect(0,0,128,12);

  _display->setColor(BLACK);
  _display->setFont(ArialMT_Plain_10);

  // menu title
  if (_menu < CONTROLLER_MENU_COUNT && _menus[_menu].name.length() > 0) {
    _display->setTextAlignment(TEXT_ALIGN_LEFT);
    _display->drawString(2, 0, _menus[_menu].name);
  }

  // RSSI indicator
  _display->setTextAlignment(TEXT_ALIGN_RIGHT);
  _display->setColor(BLACK);
  _display->setFont(TomThumb4x6);

  char rs[8];
  dtostrf(_RSSI, 2, 0, rs);
  _display->drawString(128, 3, rs);


  // battery indicator
  // TODO

  // draw menu size for debugging
  /*
  _display->setTextAlignment(TEXT_ALIGN_RIGHT);
  _display->setColor(BLACK);
  _display->setFont(TomThumb4x6);
  if (_menu < CONTROLLER_MENU_COUNT)
    _display->drawString(128, 1, String(_menus[_menu].items.size()));
  */

  // draw active menu
  _display->setColor(WHITE);
  _display->setTextAlignment(TEXT_ALIGN_LEFT);
  _display->setFont(ArialMT_Plain_10);

  switch(_menu) {
    case CONTROLLER_MENU_ROOT: manageRoot(syncMenu); break;
    case CONTROLLER_MENU_MAIN: manageMain(syncMenu); break;
    case CONTROLLER_MENU_START: manageStart(syncMenu); break;
    case CONTROLLER_MENU_CREATE: manageCreate(syncMenu); break;
    case CONTROLLER_MENU_EDIT: manageEdit(syncMenu); break;
    case CONTROLLER_MENU_BINDAXIS: manageBindAxis(syncMenu); break;
    case CONTROLLER_MENU_BINDAXIS2: manageBindAxis2(syncMenu); break;
    case CONTROLLER_MENU_BINDAXIS3: manageBindAxis3(syncMenu); break;
    case CONTROLLER_MENU_CLEAR: manageClear(syncMenu); break;
    case CONTROLLER_MENU_EDITINFO: manageEditInfo(syncMenu); break;
  }

  // write the buffer to the display
  _display->display();

  _lastMenu = _menu;
}
