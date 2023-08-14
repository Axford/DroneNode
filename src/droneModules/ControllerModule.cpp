#include "ControllerModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../DroneModuleManager.h"
#include "OLEDTomThumbFont.h"
#include "strings.h"
#include "DroneSystem.h"



ControllerModule::ControllerModule(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
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
   
   clear();

   // subs


   // outputs
   initParams(CONTROLLER_PARAM_ENTRIES);
   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = CONTROLLER_OLED_I2C_ADDRESS;
}

ControllerModule::~ControllerModule() {
  if (_display) delete _display;
}


void ControllerModule::clear() {
  Log.noticeln(F("Clearing..."));
  
  Log.noticeln(F("done"));
}


void ControllerModule::doReset() {
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  //_display->init();
  if (_display) _display->resetDisplay();

  //_display->flipScreenVertically();
}


void ControllerModule::doShutdown() {
  DroneModule::doShutdown(); // disables module

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

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



void ControllerModule::handleLinkMessage(DroneLinkMsg *msg) {

  // intercept local values
  if (msg->node() == _dlm->node()) {
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
  if (!_isBound && (msg->node() != _dlm->node())) {
    _spinner += PI / 16.0;
  }
  */
  

  /*
    Intercept other values...  TODO
  */
  
  

  DroneModule::handleLinkMessage(msg);
}


void ControllerModule::setup() {
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
 if (!_display) {
   DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

   _display = new SSD1306Wire(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0], SDA, SCL);

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

  if (!_display) return;

  //Serial.println("loop");

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);


  unsigned long loopTime = millis();


  // dim up on start
  if (_brightness < 254) _brightness+= 2;
  _display->setBrightness(_brightness);

  // clear the display
  _display->clear();

  _display->setColor(WHITE);
  _display->fillRect(0,0,128,12);

  _display->setColor(BLACK);
  _display->setFont(ArialMT_Plain_10);


  // ARM indicator
  if (_armed) {
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


  // write the buffer to the display
  _display->display();
}
