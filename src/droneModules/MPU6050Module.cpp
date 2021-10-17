#include "MPU6050Module.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

MPU6050Module::MPU6050Module(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem):
  I2CBaseModule ( id, dmm, dlm, dem )
 {
   setTypeName(FPSTR(MPU6050_STR_MPU6050));
   _addr = MPU6050_I2C_ADDRESS;

   _numParamEntries = MPU6050_PARAM_ENTRIES;
   _params = new DRONE_PARAM_ENTRY[_numParamEntries];

   // defaults
   for (uint8_t i=0; i<_numParamEntries; i++) {
     _params[i].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
     _params[i].publish = false;
     _params[i].data.f[0] = 0;
     _params[i].data.f[1] = 0;
     _params[i].data.f[2] = 0;
   }

   _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, sizeof(MPU6050_STR_MPU6050));
   strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, MPU6050_STR_MPU6050, sizeof(MPU6050_STR_MPU6050));

   // init param entries
   _params[MPU6050_PARAM_ACCEL_E].param = MPU6050_PARAM_ACCEL;
   _params[MPU6050_PARAM_ACCEL_E].name = FPSTR(STRING_ACCEL);
   _params[MPU6050_PARAM_ACCEL_E].nameLen = sizeof(STRING_ACCEL);

   _params[MPU6050_PARAM_GYRO_E].param = MPU6050_PARAM_GYRO;
   _params[MPU6050_PARAM_GYRO_E].name = FPSTR(STRING_GYRO);
   _params[MPU6050_PARAM_GYRO_E].nameLen = sizeof(STRING_GYRO);

}


void MPU6050Module::doReset() {
  I2CBaseModule::doReset();

  DroneWire::selectChannel(_bus);

  setError( _sensor.begin() ? 0 : 1 );
  if (_error) {
    Log.errorln(MPU6050_STR_MPU6050);
  }
}


void MPU6050Module::loop() {
  I2CBaseModule::loop();

  DroneWire::selectChannel(_bus);

  sensors_event_t a, g, temp;
  _sensor.getEvent(&a, &g, &temp);

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
