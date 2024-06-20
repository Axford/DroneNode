#include "HT16K33Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"


HT16K33Module::HT16K33Module(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
 {
   setTypeName(FPSTR(HT16K33_STR_HT16K33));

  // @default interval = 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;  // 1 sec

   // subs
   initSubs(HT16K33_SUBS);

   // pubs
   initParams(HT16K33_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = HT16K33_I2C_ADDRESS;

   // init param entries
}


void HT16K33Module::doReset() {
  Log.noticeln("[HT16K33.dR]");
  I2CBaseModule::doReset();
/*
  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  if (_sensor) {
    setError( _sensor->begin() ? 0 : 1 );
    if (_error) {
      Log.errorln(HT16K33_STR_HT16K33);
    }
  }*/
  Log.noticeln("[HT16K33.dR] end");
}


void HT16K33Module::setup() {
  I2CBaseModule::setup();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  // test to see if sensor responds on correct address:
  if (!DroneWire::scanAddress(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0])) {
    Log.errorln("[HT16K33] Module not detected on I2C bus");
    setError(1);
    disable();
    return;
  }

  _sensor = new HT16K33(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]);

  _sensor->begin();
  _sensor->displayOn();
  _sensor->setDigits(4);
}


void HT16K33Module::loop() {
  I2CBaseModule::loop();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  uint32_t now = millis();
  uint32_t s = now / 1000;  // convert uptime to seconds

  // calc whole minutes
  uint32_t minutes = floor(s / 60);
  uint32_t seconds = s - minutes*60;

  _sensor->displayTime(minutes, seconds, true, false);  // do not display leading zero.
  _sensor->displayColon(1);

}

