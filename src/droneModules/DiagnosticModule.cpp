#include "DiagnosticModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../DroneModuleManager.h"
#include "strings.h"
#include <SPIFFS.h>


DiagnosticModule::DiagnosticModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  I2CBaseModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(DIAGNOSTIC_STR_DIAGNOSTIC));
   _display = NULL;
   _currentModule = 0;
   _currentPage = 0;

   // subs
   initSubs(DIAGNOSTIC_SUBS);

   //DRONE_PARAM_SUB *sub;

   // pubs
   initParams(DIAGNOSTIC_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = DIAGNOSTIC_I2C_ADDRESS;

}

DiagnosticModule::~DiagnosticModule() {
  if (_display) delete _display;
}


DEM_NAMESPACE* DiagnosticModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(DIAGNOSTIC_STR_DIAGNOSTIC,0,true);
}

void DiagnosticModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {

  I2CBaseModule::registerParams(ns, dem);

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);
}

void DiagnosticModule::doReset() {
  Log.noticeln("[DM.dR]");
  I2CBaseModule::doReset();

  if (_display) _display->resetDisplay();

  Log.noticeln("[DM.dR] end");
}


void DiagnosticModule::setup() {
  I2CBaseModule::setup();

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
    _display->setBrightness(255);

    _display->setColor(WHITE);
    _display->setFont(ArialMT_Plain_10);

    _display->setTextAlignment(TEXT_ALIGN_LEFT);
    _display->drawString(2, 0, "DroneLink...");

    // write the buffer to the display
    _display->display();

    // Build list of displays to show
    for (uint8_t i=0; i<_dmm->moduleCount(); i++) {
      DroneModule* mod = _dmm->getModuleByIndex(i);
      if (mod->diagnosticDisplays() > 0) {
        _mods.add(mod);
      }
    }


  } else {
    //Serial.println("Err: _display not created");
  }
}


void DiagnosticModule::loop() {
  I2CBaseModule::loop();

  if (!_display) return;

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  // step through modules
  if (_currentModule < _mods.size()) {
    DroneModule* mod = _mods[_currentModule];

    // clear the display
    _display->clear();

    // module title
    _display->setColor(WHITE);
    _display->setFont(ArialMT_Plain_10);
    _display->setTextAlignment(TEXT_ALIGN_LEFT);
    _display->drawString(2, 0, String(_dmm->node()) + " > " + String( mod->id()) + " " + mod->getName() );

    // page number
    _display->setTextAlignment(TEXT_ALIGN_RIGHT);
    _display->drawString(126, 0, String(_currentPage + 1) );

    // render page
    _display->setTextAlignment(TEXT_ALIGN_LEFT);
    mod->drawDiagnosticDisplay(_display, _currentPage);

    _currentPage++;
    if (_currentPage >= mod->diagnosticDisplays()) {
      _currentPage = 0;
      // increment module
      _currentModule++;
      if (_currentModule >= _mods.size()) _currentModule = 0;
    }
  }

  // write the buffer to the display
  _display->display();
}
