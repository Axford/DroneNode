#include "I2CCompassModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"
#include "Preferences.h"


I2CCompassModule::I2CCompassModule(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
 {
   setTypeName(FPSTR(I2CCOMPASS_STR_I2CCOMPASS));
   _location[0] = -1.8;
   _location[1] = 52;

   _numRawSamples = 0;

   for (uint8_t i=0; i<3; i++) {
    _raw[i] = 0;
    _rawAvg[i] = 0;
    _minRaw[i] = 0;
    _maxRaw[i] = 0;
   }

   // subs
   initSubs(I2CCOMPASS_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[I2CCOMPASS_SUB_LOCATION_E];
   sub->addrParam = I2CCOMPASS_SUB_LOCATION_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, I2CCOMPASS_SUB_LOCATION);
   setParamName(FPSTR(STRING_LOCATION), &sub->param);

   sub = &_subs[I2CCOMPASS_SUB_PITCH_E];
   sub->addrParam = I2CCOMPASS_SUB_PITCH_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, I2CCOMPASS_SUB_PITCH);
   setParamName(FPSTR(STRING_PITCH), &sub->param);

   sub = &_subs[I2CCOMPASS_SUB_ROLL_E];
   sub->addrParam = I2CCOMPASS_SUB_ROLL_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, I2CCOMPASS_SUB_ROLL);
   setParamName(FPSTR(STRING_ROLL), &sub->param);

   // pubs
   initParams(I2CCOMPASS_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = I2CCOMPASS_I2C_ADDRESS;

   // init param entries
   _params[I2CCOMPASS_PARAM_VECTOR_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, I2CCOMPASS_PARAM_VECTOR);
   _params[I2CCOMPASS_PARAM_VECTOR_E].name = FPSTR(STRING_VECTOR);
   _params[I2CCOMPASS_PARAM_VECTOR_E].nameLen = sizeof(STRING_VECTOR);
   _params[I2CCOMPASS_PARAM_VECTOR_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 16);

   _params[I2CCOMPASS_PARAM_HEADING_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, I2CCOMPASS_PARAM_HEADING);
   _params[I2CCOMPASS_PARAM_HEADING_E].name = FPSTR(STRING_HEADING);
   _params[I2CCOMPASS_PARAM_HEADING_E].nameLen = sizeof(STRING_HEADING);
   _params[I2CCOMPASS_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[I2CCOMPASS_PARAM_DECLINATION_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, I2CCOMPASS_PARAM_DECLINATION);
   _params[I2CCOMPASS_PARAM_DECLINATION_E].name = FPSTR(STRING_DECLINATION);
   _params[I2CCOMPASS_PARAM_DECLINATION_E].nameLen = sizeof(STRING_DECLINATION);
   _params[I2CCOMPASS_PARAM_DECLINATION_E].data.f[0] = 0;
   _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[I2CCOMPASS_PARAM_CALIB_X_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, I2CCOMPASS_PARAM_CALIB_X);
   _params[I2CCOMPASS_PARAM_CALIB_X_E].name = FPSTR(STRING_CALIB_X);
   _params[I2CCOMPASS_PARAM_CALIB_X_E].nameLen = sizeof(STRING_CALIB_X);
   _params[I2CCOMPASS_PARAM_CALIB_X_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[0] = -1;
   _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[1] = 0;
   _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[2] = 1;

   _params[I2CCOMPASS_PARAM_CALIB_Y_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, I2CCOMPASS_PARAM_CALIB_Y);
   _params[I2CCOMPASS_PARAM_CALIB_Y_E].name = FPSTR(STRING_CALIB_Y);
   _params[I2CCOMPASS_PARAM_CALIB_Y_E].nameLen = sizeof(STRING_CALIB_Y);
   _params[I2CCOMPASS_PARAM_CALIB_Y_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[0] = -1;
   _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[1] = 0;
   _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[2] = 1;

  _params[I2CCOMPASS_PARAM_CALIB_Z_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, I2CCOMPASS_PARAM_CALIB_Z);
   _params[I2CCOMPASS_PARAM_CALIB_Z_E].name = FPSTR(STRING_CALIB_Z);
   _params[I2CCOMPASS_PARAM_CALIB_Z_E].nameLen = sizeof(STRING_CALIB_Z);
   _params[I2CCOMPASS_PARAM_CALIB_Z_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[0] = -1;
   _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[1] = 0;
   _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[2] = 1;

   _params[I2CCOMPASS_PARAM_TRIM_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, I2CCOMPASS_PARAM_TRIM);
   _params[I2CCOMPASS_PARAM_TRIM_E].name = FPSTR(STRING_TRIM);
   _params[I2CCOMPASS_PARAM_TRIM_E].nameLen = sizeof(STRING_TRIM);
   _params[I2CCOMPASS_PARAM_TRIM_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[I2CCOMPASS_PARAM_TRIM_E].data.f[0] = 0;

   _params[I2CCOMPASS_PARAM_MODE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, I2CCOMPASS_PARAM_MODE);
   _params[I2CCOMPASS_PARAM_MODE_E].name = FPSTR(STRING_MODE);
   _params[I2CCOMPASS_PARAM_MODE_E].nameLen = sizeof(STRING_MODE);
   _params[I2CCOMPASS_PARAM_MODE_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[I2CCOMPASS_PARAM_MODE_E].data.uint8[0] = I2CCOMPASS_MODE_ONLINE_CALIBRATION;

   _params[I2CCOMPASS_PARAM_RAW_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, I2CCOMPASS_PARAM_RAW);
   _params[I2CCOMPASS_PARAM_RAW_E].name = FPSTR(STRING_RAW);
   _params[I2CCOMPASS_PARAM_RAW_E].nameLen = sizeof(STRING_RAW);
   _params[I2CCOMPASS_PARAM_RAW_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 16);
}


boolean I2CCompassModule::initSensor() {
    return true;
}


void I2CCompassModule::setup() {
  I2CBaseModule::setup();

  if (initSensor()) {
    
    // load calibration values from EEPROM... if available
    Preferences pref; 
    // use module name as preference namespace
    pref.begin(_mgmtParams[DRONE_MODULE_PARAM_NAME_E].data.c, true);

    // X
    _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[0] = pref.getFloat("xMin", _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[0]);
    _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[2] = pref.getFloat("xMax", _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[2]);
    _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[1] = (_params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[2] + _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[0])/2;

    _minRaw[0] = _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[0];
    _maxRaw[0] = _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[2];

    // Y
    _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[0] = pref.getFloat("yMin", _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[0]);
    _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[2] = pref.getFloat("yMax", _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[2]);
    _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[1] = (_params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[2] + _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[0])/2;

    _minRaw[1] = _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[0];
    _maxRaw[1] = _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[2]; 
    
    // Z
    _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[0] = pref.getFloat("zMin", _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[0]);
    _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[2] = pref.getFloat("zMax", _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[2]);
    _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[1] = (_params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[2] + _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[0])/2;
    
    _minRaw[2] = _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[0];
    _maxRaw[2] = _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[2];

    pref.end();
  }
}

void I2CCompassModule::update() {
  if (!_setupDone) return;

  // called when a param is updated by handleLinkMessage

  // see if location has changed
  int newLon = round(_subs[I2CCOMPASS_SUB_LOCATION_E].param.data.f[0]);
  int newLat = round(_subs[I2CCOMPASS_SUB_LOCATION_E].param.data.f[1]);

  if (newLon != _location[0] || newLat != _location[1]) {
    _location[0] = newLon;
    _location[1] = newLat;

    // read declination value from mag.dat file
    /*
    if (SPIFFS.exists(F("/mag.dat"))) {
      File file = SPIFFS.open(F("/mag.dat"), FILE_READ);

      int minLon = -90;
      int maxLon = 0;
      int minLat = 0;
      //int maxLat = 60;

      int lonPoints = maxLon - minLon + 1;

      int mapIndex = (newLon - minLon) + (newLat - minLat) * lonPoints;

      if (file.seek(mapIndex)) {
        float decl = (file.read()-128) / 4.0f;
        _params[I2CCOMPASS_PARAM_DECLINATION_E].data.f[0] = decl;
      }

      file.close();
    }
    */
  }
}


void I2CCompassModule::updateCalibrationValuesFromRaw() {
  // update limits
  _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[0] = min(_minRaw[0], _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[0]);
  _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[2] = max(_maxRaw[0], _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[2]);
  _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[0] = min(_minRaw[1], _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[0]);
  _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[2] = max(_maxRaw[1], _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[2]);
  _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[0] = min(_minRaw[2], _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[0]);
  _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[2] = max(_maxRaw[2], _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[2]);

  // compensate for not wanting to turn the boat upside down to calibrate the compass
  // REMOVED: this caused more problems than it solved! 
  /*
  float magDia = ((_maxRaw[0] - _minRaw[0]) + (_maxRaw[1] - _minRaw[1])) / 2;
  if (_maxRaw[2] < _minRaw[2] + magDia) _maxRaw[2] = _minRaw[2] + magDia;
  _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[2] = _maxRaw[2];
  */

  // update centre
  _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[1] = (_maxRaw[0] + _minRaw[0])/2;
  _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[1] = (_maxRaw[1] + _minRaw[1])/2;
  _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[1] = (_maxRaw[2] + _minRaw[2])/2;
}


void I2CCompassModule::getSensorValues() {
  // Implement in sub-class
}


void I2CCompassModule::loop() {
  I2CBaseModule::loop();

  //Log.noticeln("I2CCOMPASS.loop");

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  getSensorValues();

  // update moving averages
  for (uint8_t i=0; i<3; i++) {
    // reject bad raw values
    if (isnan(_raw[i])) return;
    _rawAvg[i] = ( (_raw[i] / 100.0) + (_numRawSamples)*(_rawAvg[i]) ) / (_numRawSamples + 1);
  }
  
  if (_numRawSamples < I2CCOMPASS_MOVING_AVERAGE_POINTS) _numRawSamples++;

  for (uint8_t i=0; i<3; i++) {
    // update overall min and max values
    if (_rawAvg[i] > _maxRaw[i]) _maxRaw[i] = _rawAvg[i];
    if (_rawAvg[i] < _minRaw[i]) _minRaw[i] = _rawAvg[i];
  }

  // if calibrating.... 
  if (_params[I2CCOMPASS_PARAM_MODE_E].data.uint8[0] == I2CCOMPASS_MODE_ONLINE_CALIBRATION) {
    updateCalibrationValuesFromRaw();
  }
  
  _params[I2CCOMPASS_PARAM_RAW_E].data.f[0] = _rawAvg[0];
  _params[I2CCOMPASS_PARAM_RAW_E].data.f[1] = _rawAvg[1];
  _params[I2CCOMPASS_PARAM_RAW_E].data.f[2] = _rawAvg[2];
  _params[I2CCOMPASS_PARAM_RAW_E].data.f[3] = 0;

  _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[0] = _rawAvg[0];
  _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[1] = _rawAvg[1];
  _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[2] = _rawAvg[2];
  _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[3] = 0;

  // check for valid range
  if (fabs(_params[I2CCOMPASS_PARAM_VECTOR_E].data.f[0]) > 100 ||
      fabs(_params[I2CCOMPASS_PARAM_VECTOR_E].data.f[1]) > 100 ||
      fabs(_params[I2CCOMPASS_PARAM_VECTOR_E].data.f[2]) > 100) return;

  // apply pitch and roll compensation
  // start by applying fixed offset
  _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[0] = _params[I2CCOMPASS_PARAM_RAW_E].data.f[0] - _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[1];
  _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[1] = _params[I2CCOMPASS_PARAM_RAW_E].data.f[1] - _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[1];
  _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[2] = _params[I2CCOMPASS_PARAM_RAW_E].data.f[2] - _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[1];

  // calculate scaling factors, to achieve an approx unit sphere about the origin
  float xRadius = fabs(_params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[2] - _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[0])/2;
  float yRadius = fabs(_params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[2] - _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[0])/2;
  float zRadius = fabs(_params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[2] - _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[0])/2;

  // apply scaling....   raw values have now been transformed into an approximation of a unit sphere, stored in VECTOR
  _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[0] /= xRadius;
  _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[1] /= yRadius;
  _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[2] /= zRadius;

  // check to see if radii are sufficient to consider this a valid calibration
  float quality = 100 * ((xRadius/15) + (yRadius/15) + (zRadius/15))/3;
  

  // estimate quality of calibration fit by testing distance of current rawAvg from surface of calibration sphere
  float vectorLen = sqrt( 
    sq(_params[I2CCOMPASS_PARAM_VECTOR_E].data.f[0]) +
    sq(_params[I2CCOMPASS_PARAM_VECTOR_E].data.f[1]) +
    sq(_params[I2CCOMPASS_PARAM_VECTOR_E].data.f[2])
  );  

  quality = 100 * max(1 - (1 - vectorLen), 0.0f);

  _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[3] = quality;

  // if calibrating and quality is >100 (i.e sphere too small)
  // or if sphere has got too big (e.g. because of spurious max values)
  if (_params[I2CCOMPASS_PARAM_MODE_E].data.uint8[0] == I2CCOMPASS_MODE_ONLINE_CALIBRATION && (quality > 70 )) {
    /* nudge the calibration limits in the right direction
    VECTOR gives us the magnitude for each vector... we could scale that down and use it to nudge the limits directly
    */

    float nudgeScaling = 0.1 * (quality-100)/100; // bigger nudge for lower quality fit
    if (_params[I2CCOMPASS_PARAM_VECTOR_E].data.f[0] > 0) {
      _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[2] += _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[0] * nudgeScaling;
    } else {
      _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[0] += _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[0] * nudgeScaling;
    }

    if (_params[I2CCOMPASS_PARAM_VECTOR_E].data.f[1] > 0) {
      _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[2] += _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[1] * nudgeScaling;
    } else {
      _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[0] += _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[1] * nudgeScaling;
    }

    if (_params[I2CCOMPASS_PARAM_VECTOR_E].data.f[2] > 0) {
      _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[2] += _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[2] * nudgeScaling;
    } else {
      _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[0] += _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[2] * nudgeScaling;
    }
  }
    

  if (quality > 80) {
    

    // apply rotations

    // pitch
    float pitchAng = -degreesToRadians(_subs[I2CCOMPASS_SUB_PITCH_E].param.data.f[0]);
    float cosPitchAng = cos(pitchAng);
    float sinPitchAng = sin(pitchAng);
    float y1 = _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[1];
    float z1 = _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[2];
    _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[1] = y1 * cosPitchAng - z1 * sinPitchAng;
    _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[2] = y1 * sinPitchAng + z1 * cosPitchAng;

    // roll 
    float rollAng = -degreesToRadians(_subs[I2CCOMPASS_SUB_ROLL_E].param.data.f[0]);
    float cosRollAng = cos(rollAng);
    float sinRollAng = sin(rollAng);
    float x1 = _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[0];
    z1 = _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[2];
    _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[0] = x1 * cosRollAng - z1 * sinRollAng;
    _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[2] = x1 * sinRollAng + z1 * cosRollAng;
  }


  if (_params[I2CCOMPASS_PARAM_MODE_E].data.uint8[0] == I2CCOMPASS_MODE_RESET_CALIBRATION) {

    // reset raw limits
    for (uint8_t i=0; i<3; i++) {
      _minRaw[i] = _rawAvg[i]-1;
      _maxRaw[i] = _rawAvg[i]+1;
    }

    _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[0] = _rawAvg[0]-1;
    _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[2] = _rawAvg[0]+1;
    _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[0] = _rawAvg[1]-1;
    _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[2] = _rawAvg[1]+1;
    _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[0] = _rawAvg[2]-1;
    _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[2] = _rawAvg[2]+1;

    // reset calibration
    updateCalibrationValuesFromRaw();
  
    _params[I2CCOMPASS_PARAM_MODE_E].data.uint8[0] = I2CCOMPASS_MODE_ONLINE_CALIBRATION;
  }


  if (_params[I2CCOMPASS_PARAM_MODE_E].data.uint8[0] == I2CCOMPASS_MODE_STORE_CALIBRATION) {
    // write calibration values to EEPROM
    Preferences pref; 
    // use module name as preference namespace
    pref.begin(_mgmtParams[DRONE_MODULE_PARAM_NAME_E].data.c, false);

    // X
    pref.putFloat("xMin", _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[0]);
    pref.putFloat("xMax", _params[I2CCOMPASS_PARAM_CALIB_X_E].data.f[2]);

    // Y
    pref.putFloat("yMin", _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[0]);
    pref.putFloat("yMax", _params[I2CCOMPASS_PARAM_CALIB_Y_E].data.f[2]);

    // Z
    pref.putFloat("zMin", _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[0]);
    pref.putFloat("zMax", _params[I2CCOMPASS_PARAM_CALIB_Z_E].data.f[2]);

    pref.end();

    _params[I2CCOMPASS_PARAM_MODE_E].data.uint8[0] = I2CCOMPASS_MODE_FIXED_CALIBRATION;
  }

  float x = _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[0];
  float y = _params[I2CCOMPASS_PARAM_VECTOR_E].data.f[1];
  
  // calculate heading
  float heading = atan2(y, x);

  // rotate by -90 deg to account for sensor mounting orientation with y+ forward
  heading -= PI/2;

  float declinationAngle = _params[I2CCOMPASS_PARAM_DECLINATION_E].data.f[0] * PI / 180.0f; // convert to radians
  heading += declinationAngle;

  // Convert radians to degrees for readability.
  float headingDegrees = (heading * 180.0f / PI);

  // add trim
  headingDegrees += _params[I2CCOMPASS_PARAM_TRIM_E].data.f[0];

  // wrap to 0..360
  headingDegrees = fmod(headingDegrees, 360);
  if (headingDegrees < 0) headingDegrees += 360;

  _params[I2CCOMPASS_PARAM_HEADING_E].data.f[0] = headingDegrees;

  // publish param entries
  publishParamEntries();
}
