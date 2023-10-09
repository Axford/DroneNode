#include "ControllerModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../DroneModuleManager.h"
#include "OLEDTomThumbFont.h"
#include "strings.h"
#include "DroneSystem.h"


#define CM_PARSER_GENERAL 0
#define CM_PARSER_SECTION_TITLE 1
#define CM_PARSER_SECTION 2

#define CM_PARSER_NAME_BUFFER_SIZE 20
#define CM_PARSER_VALUE_BUFFER_SIZE 200


ControllerModule::ControllerModule(uint8_t id, DroneSystem *ds) : I2CBaseModule(id, ds)
{
  setTypeName(FPSTR(CONTROLLER_STR_CONTROLLER));

  // init send msg
  _sendMsg.source(_dlm->node());
  _sendMsg.type(DRONE_LINK_MSG_TYPE_FLOAT);
  _sendMsg.writable(false);
  _sendMsg.length(4);
  _sendMsg._msg.payload.f[0] = 0;

  _brightness = 0;
  _display = NULL;

  _spinner = 0;
  _scroll = 0;

  _armed = true; // TODO - make this controlled

  strcpy(_title, "Unknown");

  clear();

  // subs

  // outputs
  initParams(CONTROLLER_PARAM_ENTRIES);
  I2CBaseModule::initBaseParams();
  _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = CONTROLLER_OLED_I2C_ADDRESS;
}

ControllerModule::~ControllerModule()
{
  if (_display)
    delete _display;
}

void ControllerModule::bindSubscriptions()
{
  // create a binding for each display parameter
  for (uint8_t i = 0; i < _displayItems.size(); i++)
  {
    CONTROLLER_DISPLAY_INFO *temp = _displayItems.get(i);
    _dlm->subscribe(temp->value.node, temp->value.channel, this, temp->value.paramPriority);
  }

  // create a binding for each control parameter
  for (uint8_t i = 0; i < _controlItems.size(); i++)
  {
    CONTROLLER_CONTROL_INFO *temp = _controlItems.get(i);
    _dlm->subscribe(temp->value.node, temp->value.channel, this, temp->value.paramPriority);
  }
}

void ControllerModule::parseAddress(DRONE_LINK_ADDR *addressInfo, char * address) {
  if (addressInfo == NULL || address[0] == 0) return;

  // parse elements of address
  char buffer[20];
  uint8_t bufLen = 0;
  uint8_t part = 0;
  char c;
  uint8_t i=0;

  do {
    c = address[i];
    if (c == ' ' || c == '\t') {

    } else if (c == '>') {
      if (bufLen > 0) {
        if (buffer[0] == '@') {
          addressInfo->node = _dlm->node();
        } else {
          addressInfo->node = atoi(buffer);
        }
      } else {
        addressInfo->node = 0;
      }
      bufLen = 0;
      part = 1;
    } else if (c == '.') {
      if (bufLen > 0) {
        addressInfo->channel = atoi(buffer);
      }
      bufLen = 0;
      part = 2;

    }  else if (c == 0) {
      if (bufLen > 0) {
        addressInfo->paramPriority = atoi(buffer);
      }
      bufLen = 0;
      part = 2;

    } else {
      if (bufLen < 19) {
        buffer[bufLen] = c;
        bufLen++;
        // null terminate as we go
        buffer[bufLen] = 0;
      }
    }

    i++;
  } while (c > 0);
}


void parseDisplayPosition(CONTROLLER_DISPLAY_INFO *displayItem, char * paramList) {
  if (displayItem == NULL || paramList[0] == 0) return;

  uint8_t numValues = 0; // how many values have we parsed
  char buffer[20];
  uint8_t bufLen=0;
  // parse comma separated list of param values and set them

  uint8_t i=0;
  char c;
  do {
    c = paramList[i];

    // skip whitespace
    if (c > 0 &&
        c != ' ' && 
        c != '\t' &&
        c != ',' &&
        bufLen < 19) {
      buffer[bufLen] = c;
      bufLen++;
    } else if (c == ',' || c == 0) {
      if (bufLen > 0) {
        buffer[bufLen] = 0; // null terminate 
        displayItem->position[numValues] = atoi(buffer);
        numValues++;
        if (numValues >=2) break;
      }
      bufLen =0;
    }

    i++;
  } while (c > 0);
}


void ControllerModule::loadConfiguration(const char* filename) {

  uint8_t outerState = CM_PARSER_GENERAL;
  uint8_t inValue = false;  
  boolean inString = false;
  boolean inComment = false;
  boolean addToBuffer = false;

  uint8_t moduleId = 0;
  DroneModule *newMod = NULL;

  uint32_t line = 0;
  char valueBuffer[CM_PARSER_VALUE_BUFFER_SIZE];  // parsing buffer
  uint8_t vBufLen = 0;  // how many valid chars are in the buffer
  
  char nameBuffer[CM_PARSER_NAME_BUFFER_SIZE];
  uint8_t nBufLen = 0;

  CONTROLLER_DISPLAY_INFO *displayItem = NULL;
  CONTROLLER_CONTROL_INFO *controlItem = NULL;

  if (LITTLEFS.exists(F(filename))) {
    File file = LITTLEFS.open(F(filename), FILE_READ);

    if (!file) {
      Serial.print("[CM.lC] Error opening: ");
      Serial.println (filename);
    } else {
      
      
      while (file.available()) {
        char c = file.read();

        addToBuffer = false;

        if (!inComment) {
          if (inString) {
            // keep accumulating characters until we find a "
            if (c == '"') {
              inString = false;
            } else {
              addToBuffer = true;
            }
          } else {
            addToBuffer = (c != '\t' && 
                          c != ' ' && 
                          c != '\r' && 
                          c != '\n' &&
                          c != '=' && 
                          c != '[' &&
                          c != ']' &&
                          c != '"' &&
                          c != ';');
            if (c == '"') inString = true;
            if (c == ';') inComment = true;
          }

          // accumulate string buffer
          if (addToBuffer) {
            if (inValue) {
              if (vBufLen < CM_PARSER_VALUE_BUFFER_SIZE-1) {
                valueBuffer[vBufLen] = c;
                vBufLen++;
              }
            } else {
              if (nBufLen < CM_PARSER_NAME_BUFFER_SIZE-1) {
                nameBuffer[nBufLen] = c;
                nBufLen++;
              }
            }
          }

          if (c == '=' && !inValue) {
            inValue = true;
            vBufLen = 0;
          } else if (c == '[' && nBufLen == 0) {
            outerState = CM_PARSER_SECTION_TITLE;
            nBufLen = 0;
          } 
        }
        

        // if end of line or end of file
        if (c == '\n' || !file.available()) {
          
          // see if we have a valid name=value pair (or at least a valid name)
          if (nBufLen > 0) {
            // null terminate
            nameBuffer[nBufLen] = 0;
            valueBuffer[vBufLen] = 0;

            // do the thing
            switch(outerState) {
              case CM_PARSER_GENERAL:
                Serial.print("General: ");
                  Serial.println(nameBuffer);
                  if (strcmp(nameBuffer, "title")==0) {
                    strcpy(_title, valueBuffer);
                  }

                  break;

              case CM_PARSER_SECTION_TITLE:
                  Serial.print("Section Title: ");
                  Serial.println(nameBuffer);

                  // clear any existing items
                  displayItem = NULL;
                  controlItem = NULL;

                  // determine type of config to load
                  if (strcmp(nameBuffer, "display")==0) {
                    // prep a blank displayItem ready to populate from config
                    displayItem = (CONTROLLER_DISPLAY_INFO *)malloc(sizeof(CONTROLLER_DISPLAY_INFO));
                    displayItem->value.node = 0;
                    displayItem->value.channel = 0;
                    displayItem->value.paramPriority = 0;
                    displayItem->value.paramTypeLength = _sendMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
                    displayItem->value.payload.f[0] = 0;
                    displayItem->name[0] = 0;
                    displayItem->precision = 1;
                    displayItem->position[0] = 0;
                    displayItem->position[1] = 0;
                    _displayItems.add(displayItem);
                  } else if (strcmp(nameBuffer, "control")==0) {
                    // prep a blank controlItem ready to populate from config
                    controlItem = (CONTROLLER_CONTROL_INFO *)malloc(sizeof(CONTROLLER_CONTROL_INFO));
                    controlItem->target.node = 0;
                    controlItem->target.channel = 0;
                    controlItem->target.paramPriority = 0;
                    controlItem->value.node = 0;
                    controlItem->value.channel = 0;
                    controlItem->value.paramPriority = 0;
                    controlItem->value.paramTypeLength = _sendMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
                    controlItem->value.payload.f[0] = 0;
                    controlItem->name[0] = 0;
                    _controlItems.add(controlItem);
                  }

                  outerState = CM_PARSER_SECTION;
                  break;

              case CM_PARSER_SECTION:
                Serial.print("Section: ");
                  Serial.println(nameBuffer);

                  if (displayItem) {
                    if (strcmp(nameBuffer, "source")==0) {
                      parseAddress((DRONE_LINK_ADDR*)(&displayItem->value), valueBuffer);
                    } else if (strcmp(nameBuffer, "name")==0) {
                      strcpy(displayItem->name, valueBuffer);
                    } else if (strcmp(nameBuffer, "position")==0) {
                      parseDisplayPosition(displayItem, valueBuffer);
                    } else if (strcmp(nameBuffer, "precision")==0) {
                      displayItem->precision = atoi(valueBuffer);
                    }
                    
                  } else  if (controlItem) {
                    if (strcmp(nameBuffer, "source")==0) {
                      parseAddress((DRONE_LINK_ADDR*)(&controlItem->value), valueBuffer);
                    } else if (strcmp(nameBuffer, "target")==0) {
                      parseAddress(&controlItem->target, valueBuffer);
                    } else if (strcmp(nameBuffer, "name")==0) {
                      strcpy(controlItem->name, valueBuffer);
                    } 
                    
                  } else 
                    Log.errorln(F("[CM.lC] nothing valid to configure"));
                  break;
            }
          }
          

          nBufLen = 0;
          vBufLen = 0;
          inValue = false;
          line++;
          inString = false;
          inComment = false;
        }
      }
    }

    file.close();
  } else {
    Serial.print("[CM.lC] No: ");
    Serial.println(filename);
  }
}

void ControllerModule::clear()
{
  Log.noticeln(F("Clearing..."));

  Log.noticeln(F("done"));
}

void ControllerModule::doReset()
{
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  //_display->init();
  if (_display)
    _display->resetDisplay();

  //_display->flipScreenVertically();
}

void ControllerModule::doShutdown()
{
  DroneModule::doShutdown(); // disables module

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  // write shutdown message to screen
  // clear the display
  if (_display)
  {

    _display->clear();

    _display->setColor(WHITE);
    _display->setFont(ArialMT_Plain_10);

    _display->setTextAlignment(TEXT_ALIGN_CENTER);
    _display->drawString(64, 25, F("Restarting..."));

    // write the buffer to the display
    _display->display();
  }
}

void ControllerModule::handleLinkMessage(DroneLinkMsg *msg)
{

  // intercept local values
  if (msg->node() == _dlm->node())
  {
    // intercept values for joysticks

    // left
    /*
    uint8_t axis = 255;
    if (msg->channel() == _params[CONTROLLER_PARAM_LEFT_E].data.uint8[0]) {
      if (msg->param() >= 10 && msg->param() <= 13) {
        if (msg->type() == DRONE_LINK_MSG_TYPE_FLOAT) {
          axis = CONTROLLER_AXIS_LEFT_X + msg->param() - 10;
          _axes[ axis] = msg->_msg.payload.f[0];
        }
      }
    }
    */
  }

/*
  if ((msg->node() != _dlm->node()))
  {
    _spinner += PI / 16.0;
  }
*/

  /*
    Intercept display values
  */
  for (uint8_t i = 0; i < _displayItems.size(); i++)
  {
    CONTROLLER_DISPLAY_INFO *temp = _displayItems.get(i);

    // see if the addresses match
    if (msg->node() == temp->value.node &&
        msg->channel() == temp->value.channel &&
        msg->param() == temp->value.paramPriority && 
        msg->type() <= DRONE_LINK_MSG_TYPE_CHAR)
    {
      // update value and type
      temp->value.paramTypeLength = msg->_msg.paramTypeLength;
      uint8_t len = msg->length();
      memcpy(temp->value.payload.c, msg->_msg.payload.c, len);
      if (len < DRONE_LINK_MSG_MAX_PAYLOAD)
      {
        temp->value.payload.c[len] = 0; // ensure inbound strings are null terminated
      }

      _spinner += PI / 16.0;
    }
  }

  /*
    Intercept control values
  */
  for (uint8_t i = 0; i < _controlItems.size(); i++)
  {
    CONTROLLER_CONTROL_INFO *temp = _controlItems.get(i);

    // see if the addresses match
    if (msg->node() == temp->value.node &&
        msg->channel() == temp->value.channel &&
        msg->param() == temp->value.paramPriority && 
        msg->type() <= DRONE_LINK_MSG_TYPE_CHAR)
    {
      // update value and type
      _sendMsg._msg.paramTypeLength = msg->_msg.paramTypeLength;
      uint8_t len = msg->length();
      memcpy(_sendMsg._msg.payload.c, msg->_msg.payload.c, len);
      
      _sendMsg.writable(false);

      if (len < DRONE_LINK_MSG_MAX_PAYLOAD)
      {
        _sendMsg._msg.payload.c[len] = 0; // ensure inbound strings are null terminated
      }

      // set target address
      _sendMsg._msg.source = _dlm->node();
      _sendMsg._msg.node = temp->target.node;
      _sendMsg._msg.channel = temp->target.channel;
      _sendMsg._msg.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, temp->target.paramPriority);
      
      // publish
      Serial.print("Send ");
      Serial.print(_sendMsg._msg.payload.f[0]);
      Serial.print(" to: ");
      DroneLinkMsg::printAddress(&temp->target);
      DroneLinkMsg::printPayload(&_sendMsg._msg.payload, _sendMsg._msg.paramTypeLength);


      _dlm->sendDroneLinkMessage(temp->target.node, &_sendMsg);

      _spinner += PI / 16.0;
    }
  }

  DroneModule::handleLinkMessage(msg);
}

void ControllerModule::setup()
{
  DroneModule::setup();

  _armed = false;

  // make sure we're subscribed to the joystick channels
  /*
  _dlm->subscribe(_params[CONTROLLER_PARAM_LEFT_E].data.uint8[0], this, DRONE_LINK_PARAM_ALL);
  _dlm->subscribe(_params[CONTROLLER_PARAM_RIGHT_E].data.uint8[0], this, DRONE_LINK_PARAM_ALL);
  _dlm->subscribe(_params[CONTROLLER_PARAM_TELEMETRY_E].data.uint8[0], this, DRONE_LINK_PARAM_ALL);
  _dlm->subscribe(_params[CONTROLLER_PARAM_POWER_E].data.uint8[0], this, DRONE_LINK_PARAM_ALL);
  */

  // Init display
  if (!_display)
  {
    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

    _display = new SSD1306Wire(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0], SDA, SCL);

    if (!_display->init())
    {
      Log.errorln(F("display->init()"));
    }
    //_display->resetDisplay();

    _display->flipScreenVertically();

    _display->clear();
    _display->display();
    _display->setBrightness(_brightness);
  }
  else
  {
    // Serial.println("Err: _display not created");
  }

  // load config
  loadConfiguration("/compass.con");

  // bind subscriptions
  bindSubscriptions();

  // fire off value queries for display params
  for (uint8_t i = 0; i < _displayItems.size(); i++)
  {
    CONTROLLER_DISPLAY_INFO *temp = _displayItems.get(i);

    if (temp->value.node > 0) {
      _queryMsg.type(DRONE_LINK_MSG_TYPE_QUERY);
      _queryMsg.node(temp->value.node);
      _queryMsg.channel(temp->value.channel);
      _queryMsg.param(temp->value.paramPriority);
      _queryMsg.setUint8_t(0);
      _dlm->publish(_queryMsg);
    }
  }
}

void ControllerModule::drawSpinner()
{
  int cx = 122;
  int cy = 6;

  int x = 4 * cos(_spinner);
  int y = 4 * sin(_spinner);

  _display->setColor(BLACK);

  _display->drawLine(cx - x, cy - y, cx + x, cy + y);
}

void ControllerModule::loop()
{
  I2CBaseModule::loop();

  if (!_display)
    return;

  // Serial.println("loop");

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  unsigned long loopTime = millis();

  // dim up on start
  if (_brightness < 254)
    _brightness += 2;
  _display->setBrightness(_brightness);

  // clear the display
  _display->clear();

  _display->setColor(WHITE);
  _display->fillRect(0, 0, 128, 12);

  _display->setColor(BLACK);
  _display->setFont(ArialMT_Plain_10);
  // title
  _display->drawString(1,0, _title);

  // ARM indicator
  if (_armed)
  {
    _display->setFont(ArialMT_Plain_10);
    _display->setTextAlignment(TEXT_ALIGN_RIGHT);
    _display->drawString(111, 1, "A");
  }

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

  // draw display items
  String valueStr = "";

  for (uint8_t i = 0; i < _displayItems.size(); i++)
  {
    CONTROLLER_DISPLAY_INFO *temp = _displayItems.get(i);

    uint32_t x = temp->position[0];
    uint32_t y = 13 + temp->position[1];

    // draw label
    _display->setFont(TomThumb4x6);
    _display->setTextAlignment(TEXT_ALIGN_RIGHT);
    _display->drawString(x - 2, y + 4, temp->name);

    // draw value
    uint8_t byteLen = (temp->value.paramTypeLength & 0xF) + 1;
    uint8_t msgType = (temp->value.paramTypeLength >> 4) & 0x07;
    uint8_t numValues = (msgType == DRONE_LINK_MSG_TYPE_CHAR) ? 1 : (byteLen / DRONE_LINK_MSG_TYPE_SIZES[msgType]);

    _display->setTextAlignment(TEXT_ALIGN_LEFT);
    if (numValues > 1)
    {
      _display->setFont(TomThumb4x6);
    }
    else
      _display->setFont(ArialMT_Plain_10);

    valueStr = "";
    for (uint8_t j = 0; j < numValues; j++)
    {
      if (j > 0)
        valueStr += ' ';

      switch (msgType)
      {
      case DRONE_LINK_MSG_TYPE_UINT8_T:
        valueStr += String(temp->value.payload.uint8[j]);
        break;
      case DRONE_LINK_MSG_TYPE_UINT32_T:
        valueStr += String(temp->value.payload.uint32[j]);
        break;
      case DRONE_LINK_MSG_TYPE_FLOAT:
        valueStr += String(temp->value.payload.f[j], (unsigned int)temp->precision);
        break;
      case DRONE_LINK_MSG_TYPE_CHAR:
        valueStr += String(temp->value.payload.c);
        break;
      }
    }

    _display->drawString(x + 2, y, valueStr);
  }

  drawSpinner();

  // write the buffer to the display
  _display->display();
}
