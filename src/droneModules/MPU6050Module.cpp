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

   for (uint8_t i=0; i<3; i++) {
    _raw[i] = 0;
    _lastRaw[i] = 0;
    _rawAvg[i] = 0;
    _minRaw[i] = 0;
    _maxRaw[i] = 0;
   }

  _lastMagIndex = 0;
   for (uint8_t i=0; i<MPU6050_MOVING_AVERAGE_POINTS; i++) {
    _lastMags[i] = 0;
   }

   _magAvg = 0;
   _magVariance = 0;
   _magDSquared = 0;
   _magStdDev = 0;

   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 50;

   initParams(MPU6050_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = MPU6050_I2C_ADDRESS;
   

   DRONE_PARAM_ENTRY *param;

   // init param entries
   param = &_params[MPU6050_PARAM_ACCEL_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, MPU6050_PARAM_ACCEL);
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

   param = &_params[MPU6050_PARAM_ROLL_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, MPU6050_PARAM_ROLL);
   setParamName(FPSTR(STRING_ROLL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);  

   param = &_params[MPU6050_PARAM_RAW_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, MPU6050_PARAM_RAW);
   setParamName(FPSTR(STRING_RAW), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 16);
   param->data.f[0] = 0;
   param->data.f[1] = 0;
   param->data.f[2] = 0;
   param->data.f[3] = 0;

   param = &_params[MPU6050_PARAM_CALIB_X_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, MPU6050_PARAM_CALIB_X);
   setParamName(FPSTR(STRING_CALIB_X), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[0] = 0;
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[MPU6050_PARAM_CALIB_Y_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, MPU6050_PARAM_CALIB_Y);
   setParamName(FPSTR(STRING_CALIB_Y), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[0] = 0;
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[MPU6050_PARAM_CALIB_Z_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, MPU6050_PARAM_CALIB_Z);
   setParamName(FPSTR(STRING_CALIB_Z), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[0] = 0;
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[MPU6050_PARAM_MODE_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, MPU6050_PARAM_MODE);
   setParamName(FPSTR(STRING_MODE), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   param->data.f[0] = MPU6050_MODE_ONLINE_CALIBRATION;
}


MPU6050Module::~MPU6050Module() {
  if (_sensor) delete _sensor;
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
    _sensor->setFilterBandwidth(MPU6050_BAND_21_HZ);

    _sensor->setAccelerometerRange(MPU6050_RANGE_8_G);
  }
}


void MPU6050Module::updateCalibrationValuesFromRaw() {
  // update limits
  _params[MPU6050_PARAM_CALIB_X_E].data.f[0] = min(_minRaw[0], _params[MPU6050_PARAM_CALIB_X_E].data.f[0]);
  _params[MPU6050_PARAM_CALIB_X_E].data.f[2] = max(_maxRaw[0], _params[MPU6050_PARAM_CALIB_X_E].data.f[2]);
  _params[MPU6050_PARAM_CALIB_Y_E].data.f[0] = min(_minRaw[1], _params[MPU6050_PARAM_CALIB_Y_E].data.f[0]);
  _params[MPU6050_PARAM_CALIB_Y_E].data.f[2] = max(_maxRaw[1], _params[MPU6050_PARAM_CALIB_Y_E].data.f[2]);
  _params[MPU6050_PARAM_CALIB_Z_E].data.f[0] = min(_minRaw[2], _params[MPU6050_PARAM_CALIB_Z_E].data.f[0]);
  _params[MPU6050_PARAM_CALIB_Z_E].data.f[2] = max(_maxRaw[2], _params[MPU6050_PARAM_CALIB_Z_E].data.f[2]);

  // update centre
  _params[MPU6050_PARAM_CALIB_X_E].data.f[1] = (_maxRaw[0] + _minRaw[0])/2;
  _params[MPU6050_PARAM_CALIB_Y_E].data.f[1] = (_maxRaw[1] + _minRaw[1])/2;
  _params[MPU6050_PARAM_CALIB_Z_E].data.f[1] = (_maxRaw[2] + _minRaw[2])/2;
}


void MPU6050Module::getSensorValues() {
  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  sensors_event_t a, g, temp;
  _sensor->getEvent(&a, &g, &temp);

  // get sensor values
  _raw[0] = a.acceleration.x;
  _raw[1] = a.acceleration.y;
  _raw[2] = a.acceleration.z;

  _params[MPU6050_PARAM_GYRO_E].data.f[0] = g.gyro.x;
  _params[MPU6050_PARAM_GYRO_E].data.f[1] = g.gyro.y;
  _params[MPU6050_PARAM_GYRO_E].data.f[2] = g.gyro.z;

  _params[MPU6050_PARAM_TEMPERATURE_E].data.f[0] = temp.temperature;
}


void MPU6050Module::loop() {
  I2CBaseModule::loop();

  // fetch values and place into _raw
  getSensorValues();

  // update moving averages
  for (uint8_t i=0; i<3; i++) {
    // reject bad raw values
    if (isnan(_raw[i])) return;
    _rawAvg[i] = ( (_raw[i] ) + (MPU6050_MOVING_AVERAGE_POINTS-1)*(_rawAvg[i]) ) / (MPU6050_MOVING_AVERAGE_POINTS);
    //_rawAvg[i] = _rawAvg[i] + (_raw[i] - _lastRaw[i]) / MPU6050_MOVING_AVERAGE_POINTS;
    _lastRaw[i] = _raw[i];
  }

  // calculate magnitude of accel vector
  float mag = sqrt(sq(_raw[0]) + sq(_raw[1]) + sq(_raw[2]));

  // add new mag reading to _lastMags buffer
  float oldMag = _lastMags[_lastMagIndex];
  _lastMags[_lastMagIndex] = mag;
  _lastMagIndex = (_lastMagIndex+1) % MPU6050_MOVING_AVERAGE_POINTS;

  // calulate mean of _lastMags
  _magAvg = 0;
  for (uint8_t i=0; i<MPU6050_MOVING_AVERAGE_POINTS; i++) {
    _magAvg += _lastMags[i];
  }
  _magAvg /= MPU6050_MOVING_AVERAGE_POINTS;

  // calculate square diffs
  _magVariance = 0;
  for (uint8_t i=0; i<MPU6050_MOVING_AVERAGE_POINTS; i++) {
    float sqDiff = _lastMags[i] - _magAvg;
    _magVariance += (sqDiff * sqDiff);
  }
  _magVariance /= MPU6050_MOVING_AVERAGE_POINTS;

  _magStdDev = sqrt( _magVariance );

  /*
  initial testing suggests a stdDev of <0.025 represents a static condition
  */
  if (_params[MPU6050_PARAM_MODE_E].data.uint8[0] == MPU6050_MODE_ONLINE_CALIBRATION &&
      (_magStdDev < 0.02)) {
    for (uint8_t i=0; i<3; i++) {
      // update overall min and max values
      if (_rawAvg[i] > _maxRaw[i]) _maxRaw[i] = _rawAvg[i];
      if (_rawAvg[i] < _minRaw[i]) _minRaw[i] = _rawAvg[i];
    }

    updateCalibrationValuesFromRaw();
  }
  
  // copy _raw values into param
  for (uint8_t i=0; i<3; i++) {
    _params[MPU6050_PARAM_ACCEL_E].data.f[i] = _raw[i];
    _params[MPU6050_PARAM_RAW_E].data.f[i] = _rawAvg[i];
  }
  //_params[MPU6050_PARAM_RAW_E].data.f[0] = mag;
  //_params[MPU6050_PARAM_RAW_E].data.f[1] = _magAvg;
  //_params[MPU6050_PARAM_RAW_E].data.f[2] = _magVariance;
  _params[MPU6050_PARAM_RAW_E].data.f[3] = _magStdDev;
  

  // calc pitch - assume standard orientation with Y+ forward
  //float mag = sqrt(SQR(a.acceleration.x) + SQR(a.acceleration.y) + SQR(a.acceleration.z));

  float pitch = atan2(_raw[1], _raw[2]) * 180 / PI;
  _params[MPU6050_PARAM_PITCH_E].data.f[0] = pitch;

  // positive roll is to the right, negative to the left
  float roll = atan2(_raw[0], _raw[2]) * 180 / PI;
  _params[MPU6050_PARAM_ROLL_E].data.f[0] = roll;

  // error check
  /*
  if (isnan(_params[MPU6050_PARAM_ACCEL_E].data.f[0])) {
    setError(1);  // will be cleared by next watchdog
  }
  */

  // publish param entries
  publishParamEntries();
}
