#include "MPU6050Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

#define SQR(x) (x*x) 

MPU6050Module::MPU6050Module(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
 {
   setTypeName(FPSTR(MPU6050_STR_MPU6050));
   _sensor = NULL;

   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;  

   initParams(MPU6050_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = MPU6050_I2C_ADDRESS;
   

   DRONE_PARAM_ENTRY *param;

   // init param entries
   param = &_params[MPU6050_PARAM_ACCEL_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, MPU6050_PARAM_ACCEL);
   setParamName(FPSTR(STRING_ACCEL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[0] = 0;
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[MPU6050_PARAM_GYRO_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, MPU6050_PARAM_GYRO);
   setParamName(FPSTR(STRING_GYRO), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[0] = 0;
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[MPU6050_PARAM_TEMPERATURE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, MPU6050_PARAM_TEMPERATURE);
   setParamName(FPSTR(STRING_TEMPERATURE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[MPU6050_PARAM_PITCH_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, MPU6050_PARAM_PITCH);
   setParamName(FPSTR(STRING_PITCH), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);  
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
    if (!_sensor->begin()) {
      setError(1);
      disable();
      return;
    }

    // configure
    _sensor->setFilterBandwidth(MPU6050_BAND_5_HZ);

    _sensor->setAccelerometerRange(MPU6050_RANGE_8_G);
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

  _params[MPU6050_PARAM_TEMPERATURE_E].data.f[0] = temp.temperature;

  // calc pitch - assume standard orientation with Y+ forward
  //float mag = sqrt(SQR(a.acceleration.x) + SQR(a.acceleration.y) + SQR(a.acceleration.z));

  float pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  _params[MPU6050_PARAM_PITCH_E].data.f[0] = pitch;

  // error check
  if (isnan(_params[MPU6050_PARAM_ACCEL_E].data.f[0])) {
    setError(1);  // will be cleared by next watchdog
  }

  // publish param entries
  publishParamEntries();
}
