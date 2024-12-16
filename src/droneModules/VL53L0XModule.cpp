#include "VL53L0XModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include <LittleFS.h>


VL53L0XModule::VL53L0XModule(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
 {
   setTypeName(FPSTR(VL53L0X_STR_VL53L0X));
   
   // @default interval = 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;  // 1 sec

   // subs
   initSubs(VL53L0X_SUBS);

   DRONE_PARAM_SUB *sub;

   // pubs
   initParams(VL53L0X_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = VL53L0X_I2C_ADDRESS;

   // init param entries
   _params[VL53L0X_PARAM_DISTANCE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, VL53L0X_PARAM_DISTANCE);
   _params[VL53L0X_PARAM_DISTANCE_E].name = FPSTR(STRING_DISTANCE);
   _params[VL53L0X_PARAM_DISTANCE_E].nameLen = sizeof(STRING_DISTANCE);
   _params[VL53L0X_PARAM_DISTANCE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
}


void VL53L0XModule::doReset() {
  I2CBaseModule::doReset();
/*
  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  if (_sensor) {
    setError( _sensor->begin() ? 0 : 1 );
    if (_error) {
      Log.errorln(VL53L0X_STR_VL53L0X);
    }
  }*/
}


void VL53L0XModule::setup() {
  I2CBaseModule::setup();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  // test to see if sensor responds on correct address:
  if (!DroneWire::scanAddress(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0])) {
    Log.errorln("[VL53L0X] Module not detected on I2C bus");
    setError(1);
    disable();
    return;
  }

  // init sensor
  _sensor = new VL53L0X();

  _sensor->setTimeout(500);


  if (!_sensor->init()) {
    Log.errorln("[VL53L0X] Module failed to init");
    setError(1);
    disable();
    return;
  }

  _sensor->startContinuous(_mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0]);
}


void VL53L0XModule::loop() {
  I2CBaseModule::loop();


  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  float f = _sensor->readRangeContinuousMillimeters();

  if (!_sensor->timeoutOccurred()) {
    updateAndPublishParam(&_params[VL53L0X_PARAM_DISTANCE_E], (uint8_t*)&f, sizeof(f));
  }
}

