#include "MPU6050Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

MPU6050Module::MPU6050Module(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  I2CBaseModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(MPU6050_STR_MPU6050));
   _sensor = NULL;

   initParams(MPU6050_PARAM_ENTRIES);

   // defaults
   for (uint8_t i=0; i<_numParamEntries; i++) {
     _params[i].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
     _params[i].publish = false;
     _params[i].data.f[0] = 0;
     _params[i].data.f[1] = 0;
     _params[i].data.f[2] = 0;
   }

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = MPU6050_I2C_ADDRESS;

   // init param entries
   _params[MPU6050_PARAM_ACCEL_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, MPU6050_PARAM_ACCEL);
   _params[MPU6050_PARAM_ACCEL_E].name = FPSTR(STRING_ACCEL);
   _params[MPU6050_PARAM_ACCEL_E].nameLen = sizeof(STRING_ACCEL);

   _params[MPU6050_PARAM_GYRO_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, MPU6050_PARAM_GYRO);
   _params[MPU6050_PARAM_GYRO_E].name = FPSTR(STRING_GYRO);
   _params[MPU6050_PARAM_GYRO_E].nameLen = sizeof(STRING_GYRO);

}


MPU6050Module::~MPU6050Module() {
  if (_sensor) delete _sensor;
}


DEM_NAMESPACE* MPU6050Module::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(MPU6050_STR_MPU6050,0,true);
}

void MPU6050Module::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  I2CBaseModule::registerParams(ns, dem);

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);

  //dem->registerCommand(ns, STRING_THRESHOLD, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}


void MPU6050Module::doReset() {
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  if (_sensor) {
    setError( _sensor->begin() ? 0 : 1 );
    if (_error) {
      Log.errorln(MPU6050_STR_MPU6050);
    }
  }
}


void MPU6050Module::setup() {
  I2CBaseModule::setup();
  // instantiate sensor object, now _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] is known
  if (!_sensor) {
    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
    _sensor = new Adafruit_MPU6050();
    _sensor->begin();
  }
}


void MPU6050Module::loop() {
  I2CBaseModule::loop();

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  sensors_event_t a, g, temp;
  _sensor->getEvent(&a, &g, &temp);

  // get sensor values
  _params[MPU6050_PARAM_ACCEL_E].data.f[0] = a.acceleration.x;
  _params[MPU6050_PARAM_ACCEL_E].data.f[1] = a.acceleration.y;
  _params[MPU6050_PARAM_ACCEL_E].data.f[2] = a.acceleration.z;

  _params[MPU6050_PARAM_GYRO_E].data.f[0] = g.gyro.x;
  _params[MPU6050_PARAM_GYRO_E].data.f[1] = g.gyro.y;
  _params[MPU6050_PARAM_GYRO_E].data.f[2] = g.gyro.z;


  // error check
  if (isnan(_params[MPU6050_PARAM_ACCEL_E].data.f[0])) {
    setError(1);  // will be cleared by next watchdog
  }

  // publish param entries
  publishParamEntries();
}
