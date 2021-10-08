#include "OLEDModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../DroneModuleManager.h"
#include "OLEDTomThumbFont.h"

OLEDModule::OLEDModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  I2CBaseModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(OLED_STR_OLED));
   _addr = OLED_I2C_ADDRESS;

   // init sub labels
   for (uint8_t i=0; i<OLED_NUM_LABELS; i++) {
     _subLabels[i][0] = '0' + i;
     _subLabels[i][1] = '\0';
     _labelState[i] = notNeeded;
   }

   // init query msg
   _queryMsg.type(DRONE_LINK_MSG_TYPE_NAMEQUERY);
   _queryMsg.length(1);
   _queryMsg._msg.payload.uint8[0] = 0;

   // subs
   initSubs(OLED_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[OLED_SUB_SUB1_E];
   sub->addrParam = OLED_SUB_SUB1_ADDR;
   sub->param.param = OLED_SUB_SUB1;
   setParamName(FPSTR(DRONE_STR_SUB1), &sub->param);

   sub = &_subs[OLED_SUB_SUB2_E];
   sub->addrParam = OLED_SUB_SUB2_ADDR;
   sub->param.param = OLED_SUB_SUB2;
   setParamName(FPSTR(DRONE_STR_SUB2), &sub->param);

   sub = &_subs[OLED_SUB_SUB3_E];
   sub->addrParam = OLED_SUB_SUB3_ADDR;
   sub->param.param = OLED_SUB_SUB3;
   setParamName(FPSTR(DRONE_STR_SUB3), &sub->param);

   sub = &_subs[OLED_SUB_SUB4_E];
   sub->addrParam = OLED_SUB_SUB4_ADDR;
   sub->param.param = OLED_SUB_SUB4;
   setParamName(FPSTR(DRONE_STR_SUB4), &sub->param);
}

OLEDModule::~OLEDModule() {
  if (_display) delete _display;
}


void OLEDModule::doReset() {
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_bus);

  _display->init();
  //_display->resetDisplay();

  _display->flipScreenVertically();
  _display->setFont(ArialMT_Plain_10);
}


void OLEDModule::handleLinkMessage(DroneLinkMsg *msg) {
  DroneModule::handleLinkMessage(msg);

  // intercept values for our subs
  for (uint8_t i=0; i<_numSubs; i++) {
    if (msg->sameAddress(&_subs[i].addr)) {
      if (msg->type() == DRONE_LINK_MSG_TYPE_NAME) {
        strncpy(_subLabels[i], msg->_msg.payload.c, msg->length());
        _subLabels[i][msg->length()] = '\0';
        _labelState[i] = received;
      } else if (msg->type() <= DRONE_LINK_MSG_TYPE_CHAR) {
        // match type
        _subs[i].param.paramTypeLength = msg->_msg.paramTypeLength & (~DRONE_LINK_MSG_WRITABLE);
        // handle the subscription updating
        memcpy(_subs[i].param.data.c, msg->_msg.payload.c, msg->length());

        // ensure strings are null terminated
        if (msg->type() == DRONE_LINK_MSG_TYPE_CHAR && msg->length() < 16) {
          _subs[i].param.data.c[msg->length()] = '\0';
        }

        _subs[i].received = true;
      }
    }
  }
}


void OLEDModule::loadConfiguration(JsonObject &obj) {
  I2CBaseModule::loadConfiguration(obj);

  // instantiate sensor object, now _addr is known
  _display = new SSD1306Wire(_addr, SDA, SCL);
}


void OLEDModule::loop() {
  I2CBaseModule::loop();

  DroneWire::selectChannel(_bus);

  // clear the display
  _display->clear();

  _display->setColor(WHITE);
  _display->fillRect(0,0,128,12);

  _display->setColor(BLACK);
  _display->setFont(ArialMT_Plain_10);

  // node id
  _display->setTextAlignment(TEXT_ALIGN_RIGHT);
  _display->drawString(58, 0, String(_dmm->node()));

  // hostname
  _display->setTextAlignment(TEXT_ALIGN_LEFT);
  _display->drawString(62, 0, _dmm->hostname());

  _display->setColor(WHITE);

  // sub1
  uint8_t y = 14;
  boolean labelRequested = false;
  String valueStr;

  for (uint8_t i=0; i<OLED_NUM_LABELS; i++) {
    // ensure we've received a message
    if (_subs[i].received) {
      // trigger discovery
      if (_labelState[i] == notNeeded) _labelState[i] = needed;

      if (_labelState[i] == requested) labelRequested = true;

      if (_labelState[i] == needed && labelRequested == false) {
        // send a namequery message
        _labelState[i] = requested;
        _queryMsg._msg.node = _subs[i].addr.node;
        _queryMsg._msg.channel = _subs[i].addr.channel;
        _queryMsg._msg.param = _subs[i].addr.param;
        _dlm->publish(_queryMsg);
      }

      // label
      _display->setFont(TomThumb4x6);
      _display->drawString(0, y+4, String(_subs[i].addr.node) + '>' + String(_subs[i].addr.channel));
      _display->setTextAlignment(TEXT_ALIGN_RIGHT);
      _display->drawString(58, y+5, _subLabels[i]);

      // value
      uint8_t byteLen = (_subs[i].param.paramTypeLength & 0xF)+1;
      uint8_t msgType = (_subs[i].param.paramTypeLength >> 4) & 0x07;
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
            valueStr += String(_subs[i].param.data.uint8[j]);
            break;
          case DRONE_LINK_MSG_TYPE_UINT32_T:
            valueStr += String(_subs[i].param.data.uint32[j]);
            break;
          case DRONE_LINK_MSG_TYPE_FLOAT:
            valueStr += String(_subs[i].param.data.f[j]);
            break;
          case DRONE_LINK_MSG_TYPE_CHAR:
            valueStr += String(_subs[i].param.data.c);
            break;
        }
      }

      _display->drawString(62, y + (numValues > 1 ? 5 : 0), valueStr);

      y += 12;
    }

  }


  // write the buffer to the display
  _display->display();

}
