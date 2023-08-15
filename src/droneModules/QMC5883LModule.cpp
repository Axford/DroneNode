#include "QMC5883LModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include <SPIFFS.h>
#include "../navMath.h"
#include "Preferences.h"

#define SQR(x) ((x)*(x))


QMC5883LModule::QMC5883LModule(uint8_t id, DroneSystem* ds):
  I2CBaseModule ( id, ds )
 {
   setTypeName(FPSTR(QMC5883L_STR_QMC5883L));
   _sensor = NULL;
   _location[0] = -1.8;
   _location[1] = 52;

   _numRawSamples = 0;

   for (uint8_t i=0; i<3; i++) {
    _minRaw[i] = 0;
    _maxRaw[i] = 0;
   }

   // subs
   initSubs(QMC5883L_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[QMC5883L_SUB_LOCATION_E];
   sub->addrParam = QMC5883L_SUB_LOCATION_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_SUB_LOCATION);
   setParamName(FPSTR(STRING_LOCATION), &sub->param);

   sub = &_subs[QMC5883L_SUB_PITCH_E];
   sub->addrParam = QMC5883L_SUB_PITCH_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_SUB_PITCH);
   setParamName(FPSTR(STRING_PITCH), &sub->param);

   sub = &_subs[QMC5883L_SUB_ROLL_E];
   sub->addrParam = QMC5883L_SUB_ROLL_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_SUB_ROLL);
   setParamName(FPSTR(STRING_ROLL), &sub->param);

   // pubs
   initParams(QMC5883L_PARAM_ENTRIES);

   I2CBaseModule::initBaseParams();
   _params[I2CBASE_PARAM_ADDR_E].data.uint8[0] = QMC5883L_I2C_ADDRESS;

   // init param entries
   _params[QMC5883L_PARAM_VECTOR_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, QMC5883L_PARAM_VECTOR);
   _params[QMC5883L_PARAM_VECTOR_E].name = FPSTR(STRING_VECTOR);
   _params[QMC5883L_PARAM_VECTOR_E].nameLen = sizeof(STRING_VECTOR);
   _params[QMC5883L_PARAM_VECTOR_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 16);

   _params[QMC5883L_PARAM_HEADING_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, QMC5883L_PARAM_HEADING);
   _params[QMC5883L_PARAM_HEADING_E].name = FPSTR(STRING_HEADING);
   _params[QMC5883L_PARAM_HEADING_E].nameLen = sizeof(STRING_HEADING);
   _params[QMC5883L_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[QMC5883L_PARAM_DECLINATION_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_PARAM_DECLINATION);
   _params[QMC5883L_PARAM_DECLINATION_E].name = FPSTR(STRING_DECLINATION);
   _params[QMC5883L_PARAM_DECLINATION_E].nameLen = sizeof(STRING_DECLINATION);
   _params[QMC5883L_PARAM_DECLINATION_E].data.f[0] = 0;
   _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[QMC5883L_PARAM_CALIB_X_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, QMC5883L_PARAM_CALIB_X);
   _params[QMC5883L_PARAM_CALIB_X_E].name = FPSTR(STRING_CALIB_X);
   _params[QMC5883L_PARAM_CALIB_X_E].nameLen = sizeof(STRING_CALIB_X);
   _params[QMC5883L_PARAM_CALIB_X_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[QMC5883L_PARAM_CALIB_X_E].data.f[0] = -1;
   _params[QMC5883L_PARAM_CALIB_X_E].data.f[1] = 0;
   _params[QMC5883L_PARAM_CALIB_X_E].data.f[2] = 1;

   _params[QMC5883L_PARAM_CALIB_Y_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, QMC5883L_PARAM_CALIB_Y);
   _params[QMC5883L_PARAM_CALIB_Y_E].name = FPSTR(STRING_CALIB_Y);
   _params[QMC5883L_PARAM_CALIB_Y_E].nameLen = sizeof(STRING_CALIB_Y);
   _params[QMC5883L_PARAM_CALIB_Y_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[QMC5883L_PARAM_CALIB_Y_E].data.f[0] = -1;
   _params[QMC5883L_PARAM_CALIB_Y_E].data.f[1] = 0;
   _params[QMC5883L_PARAM_CALIB_Y_E].data.f[2] = 1;

  _params[QMC5883L_PARAM_CALIB_Z_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, QMC5883L_PARAM_CALIB_Z);
   _params[QMC5883L_PARAM_CALIB_Z_E].name = FPSTR(STRING_CALIB_Z);
   _params[QMC5883L_PARAM_CALIB_Z_E].nameLen = sizeof(STRING_CALIB_Z);
   _params[QMC5883L_PARAM_CALIB_Z_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   _params[QMC5883L_PARAM_CALIB_Z_E].data.f[0] = -1;
   _params[QMC5883L_PARAM_CALIB_Z_E].data.f[1] = 0;
   _params[QMC5883L_PARAM_CALIB_Z_E].data.f[2] = 1;

   _params[QMC5883L_PARAM_TRIM_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_PARAM_TRIM);
   _params[QMC5883L_PARAM_TRIM_E].name = FPSTR(STRING_TRIM);
   _params[QMC5883L_PARAM_TRIM_E].nameLen = sizeof(STRING_TRIM);
   _params[QMC5883L_PARAM_TRIM_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[QMC5883L_PARAM_TRIM_E].data.f[0] = 0;

   _params[QMC5883L_PARAM_MODE_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, QMC5883L_PARAM_MODE);
   _params[QMC5883L_PARAM_MODE_E].name = FPSTR(STRING_MODE);
   _params[QMC5883L_PARAM_MODE_E].nameLen = sizeof(STRING_MODE);
   _params[QMC5883L_PARAM_MODE_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[QMC5883L_PARAM_MODE_E].data.uint8[0] = QMC5883L_MODE_ONLINE_CALIBRATION;

   _params[QMC5883L_PARAM_RAW_E].paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, QMC5883L_PARAM_RAW);
   _params[QMC5883L_PARAM_RAW_E].name = FPSTR(STRING_RAW);
   _params[QMC5883L_PARAM_RAW_E].nameLen = sizeof(STRING_RAW);
   _params[QMC5883L_PARAM_RAW_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 16);
}

QMC5883LModule::~QMC5883LModule() {
  if (_sensor) delete _sensor;
}


void QMC5883LModule::doReset() {
  Log.noticeln("[HMC.dR]");
  I2CBaseModule::doReset();
/*
  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  if (_sensor) {
    setError( _sensor->begin() ? 0 : 1 );
    if (_error) {
      Log.errorln(QMC5883L_STR_QMC5883L);
    }
  }*/
  Log.noticeln("[HMC.dR] end");
}


void QMC5883LModule::setup() {
  I2CBaseModule::setup();

  if (!_sensor) {
    DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);
    /*_sensor = new Adafruit_QMC5883_Unified(_params[I2CBASE_PARAM_ADDR_E].data.uint8[0]);
    if (!_sensor->begin() ){
      Log.errorln("Failed to init QMC5883L");
    }*/
    _sensor = new QMC5883LCompass();
    _sensor->init();
    
    // load calibration values from EEPROM... if available
    Preferences pref; 
    // use module name as preference namespace
    pref.begin(_mgmtParams[DRONE_MODULE_PARAM_NAME_E].data.c, true);

    // X
    _params[QMC5883L_PARAM_CALIB_X_E].data.f[0] = pref.getFloat("xMin", _params[QMC5883L_PARAM_CALIB_X_E].data.f[0]);
    _params[QMC5883L_PARAM_CALIB_X_E].data.f[2] = pref.getFloat("xMax", _params[QMC5883L_PARAM_CALIB_X_E].data.f[2]);
    _params[QMC5883L_PARAM_CALIB_X_E].data.f[1] = (_params[QMC5883L_PARAM_CALIB_X_E].data.f[2] + _params[QMC5883L_PARAM_CALIB_X_E].data.f[0])/2;

    _minRaw[0] = _params[QMC5883L_PARAM_CALIB_X_E].data.f[0];
    _maxRaw[0] = _params[QMC5883L_PARAM_CALIB_X_E].data.f[2];

    // Y
    _params[QMC5883L_PARAM_CALIB_Y_E].data.f[0] = pref.getFloat("yMin", _params[QMC5883L_PARAM_CALIB_Y_E].data.f[0]);
    _params[QMC5883L_PARAM_CALIB_Y_E].data.f[2] = pref.getFloat("yMax", _params[QMC5883L_PARAM_CALIB_Y_E].data.f[2]);
    _params[QMC5883L_PARAM_CALIB_Y_E].data.f[1] = (_params[QMC5883L_PARAM_CALIB_Y_E].data.f[2] + _params[QMC5883L_PARAM_CALIB_Y_E].data.f[0])/2;

    _minRaw[1] = _params[QMC5883L_PARAM_CALIB_Y_E].data.f[0];
    _maxRaw[1] = _params[QMC5883L_PARAM_CALIB_Y_E].data.f[2]; 
    
    // Z
    _params[QMC5883L_PARAM_CALIB_Z_E].data.f[0] = pref.getFloat("zMin", _params[QMC5883L_PARAM_CALIB_Z_E].data.f[0]);
    _params[QMC5883L_PARAM_CALIB_Z_E].data.f[2] = pref.getFloat("zMax", _params[QMC5883L_PARAM_CALIB_Z_E].data.f[2]);
    _params[QMC5883L_PARAM_CALIB_Z_E].data.f[1] = (_params[QMC5883L_PARAM_CALIB_Z_E].data.f[2] + _params[QMC5883L_PARAM_CALIB_Z_E].data.f[0])/2;
    
    _minRaw[2] = _params[QMC5883L_PARAM_CALIB_Z_E].data.f[0];
    _maxRaw[2] = _params[QMC5883L_PARAM_CALIB_Z_E].data.f[2];

    pref.end();
  }
}

void QMC5883LModule::update() {
  if (!_setupDone) return;

  // called when a param is updated by handleLinkMessage

  // see if location has changed
  int newLon = round(_subs[QMC5883L_SUB_LOCATION_E].param.data.f[0]);
  int newLat = round(_subs[QMC5883L_SUB_LOCATION_E].param.data.f[1]);

  if (newLon != _location[0] || newLat != _location[1]) {
    _location[0] = newLon;
    _location[1] = newLat;

    // read declination value from mag.dat file
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
        _params[QMC5883L_PARAM_DECLINATION_E].data.f[0] = decl;
      }

      file.close();
    }
  }
}

/*
boolean QMC5883LModule::addSamplePoint(float x, float y, float z) {
  float targetDistance = 2 * PI * 6 / QMC5883L_MAX_SAMPLE_POINTS; 

  float minDist = 1000;

  // check distance to existing points, only evaluate x and y for now
  for (uint8_t i=0; i<_numSamples; i++) {
    float d = (SQR(x - _samples[i].x) + SQR(y - _samples[i].y));
    if (d < minDist) minDist = d;
  }
  minDist = sqrt(minDist);

  float xScale = fabs(_params[QMC5883L_PARAM_LIMITS_E].data.f[1] - _params[QMC5883L_PARAM_LIMITS_E].data.f[3])/2;
  float yScale = fabs(_params[QMC5883L_PARAM_LIMITS_E].data.f[0]  - _params[QMC5883L_PARAM_LIMITS_E].data.f[2])/2;

  // we expect xScale and yScale to be around 6
  boolean sufficientScale = (xScale > 4) && (yScale > 4);

  if (!sufficientScale || (minDist > targetDistance * 0.8 && minDist < targetDistance * 1.4)) {

    int newIndex = -1;

    // see if we've maxed out our samples
    if (_numSamples == QMC5883L_MAX_SAMPLE_POINTS-1 && xScale > 0 && yScale > 0) {
      // check distance of each sample point from the estimated centre
      for (uint8_t i=0; i<_numSamples; i++) {
        // convert to circle - translate and scale
        float x = (_samples[i].x - _params[QMC5883L_PARAM_CALIB_X_E].data.f[1]) / xScale;
        float y = (_samples[i].y - _params[QMC5883L_PARAM_CALIB_Y_E].data.f[1]) / yScale;
        float d = (SQR(x) + SQR(y));
        // aim to prune points that are a poor fit to our best estimate
        if (d < 0.8 || d > 1.2) {
          newIndex = i;
          break;
        } 
      }
    } else if (newIndex == -1 && (_numSamples < QMC5883L_MAX_SAMPLE_POINTS-1)) {
      newIndex = _numSamples;
      _numSamples++;
    }

    // add to samples
    if (newIndex > -1 && newIndex < _numSamples) {
      _samples[newIndex].x = x;
      _samples[newIndex].y = y;
      _samples[newIndex].z = z;

      _params[QMC5883L_PARAM_SAMPLES_E].data.uint32[0] = _numSamples;

      return true;
    }
  }

  return false;
}
*/


void QMC5883LModule::loop() {
  I2CBaseModule::loop();

  //Log.noticeln("QMC5883L.loop");

  DroneWire::selectChannel(_params[I2CBASE_PARAM_BUS_E].data.uint8[0]);

  // get sensor values
  _sensor->read();

  float sv[3] = { _sensor->getX(), _sensor->getY(), _sensor->getZ() };

  // update moving averages
  for (uint8_t i=0; i<3; i++) {
    // reject bad raw values
    if (isnan(sv[i])) return;
    _rawAvg[i] = ( (sv[i] / 100.0) + (_numRawSamples)*(_rawAvg[i]) ) / (_numRawSamples + 1);
  }
  
  if (_numRawSamples < QMC5883L_MOVING_AVERAGE_POINTS) _numRawSamples++;

  for (uint8_t i=0; i<3; i++) {
    // update overall min and max values
    if (_rawAvg[i] > _maxRaw[i]) _maxRaw[i] = _rawAvg[i];
    if (_rawAvg[i] < _minRaw[i]) _minRaw[i] = _rawAvg[i];
  }

  // if calibrating.... 
  if (_params[QMC5883L_PARAM_MODE_E].data.uint8[0] == QMC5883L_MODE_ONLINE_CALIBRATION) {
    // update limits
    _params[QMC5883L_PARAM_CALIB_X_E].data.f[0] = _minRaw[0];
    _params[QMC5883L_PARAM_CALIB_X_E].data.f[2] = _maxRaw[0];
    _params[QMC5883L_PARAM_CALIB_Y_E].data.f[0] = _minRaw[1];
    _params[QMC5883L_PARAM_CALIB_Y_E].data.f[2] = _maxRaw[1];
    _params[QMC5883L_PARAM_CALIB_Z_E].data.f[0] = _minRaw[2];
    _params[QMC5883L_PARAM_CALIB_Z_E].data.f[2] = _maxRaw[2];

    // compensate for not wanting to turn the boat upside down to calibrate the compass
    float magDia = ((_maxRaw[0] - _minRaw[0]) + (_maxRaw[1] - _minRaw[1])) / 2;
    if (_maxRaw[2] < _minRaw[2] + magDia) _maxRaw[2] = _minRaw[2] + magDia;
    _params[QMC5883L_PARAM_CALIB_Z_E].data.f[2] = _maxRaw[2];

    // update centre
    _params[QMC5883L_PARAM_CALIB_X_E].data.f[1] = (_maxRaw[0] + _minRaw[0])/2;
    _params[QMC5883L_PARAM_CALIB_Y_E].data.f[1] = (_maxRaw[1] + _minRaw[1])/2;
    _params[QMC5883L_PARAM_CALIB_Z_E].data.f[1] = (_maxRaw[2] + _minRaw[2])/2;
  }
  
  _params[QMC5883L_PARAM_RAW_E].data.f[0] = _rawAvg[0];
  _params[QMC5883L_PARAM_RAW_E].data.f[1] = _rawAvg[1];
  _params[QMC5883L_PARAM_RAW_E].data.f[2] = _rawAvg[2];
  _params[QMC5883L_PARAM_RAW_E].data.f[3] = 0;

  _params[QMC5883L_PARAM_VECTOR_E].data.f[0] = _rawAvg[0];
  _params[QMC5883L_PARAM_VECTOR_E].data.f[1] = _rawAvg[1];
  _params[QMC5883L_PARAM_VECTOR_E].data.f[2] = _rawAvg[2];
  _params[QMC5883L_PARAM_VECTOR_E].data.f[3] = 0;

  // check for valid range
  if (fabs(_params[QMC5883L_PARAM_VECTOR_E].data.f[0]) > 100 ||
      fabs(_params[QMC5883L_PARAM_VECTOR_E].data.f[1]) > 100 ||
      fabs(_params[QMC5883L_PARAM_VECTOR_E].data.f[2]) > 100) return;

  // apply pitch and roll compensation
  // start by applying fixed offset
  _params[QMC5883L_PARAM_VECTOR_E].data.f[0] = _params[QMC5883L_PARAM_RAW_E].data.f[0] - _params[QMC5883L_PARAM_CALIB_X_E].data.f[1];
  _params[QMC5883L_PARAM_VECTOR_E].data.f[1] = _params[QMC5883L_PARAM_RAW_E].data.f[1] - _params[QMC5883L_PARAM_CALIB_Y_E].data.f[1];
  _params[QMC5883L_PARAM_VECTOR_E].data.f[2] = _params[QMC5883L_PARAM_RAW_E].data.f[2] - _params[QMC5883L_PARAM_CALIB_Z_E].data.f[1];

  // calculate scaling factors, to achieve an approx unit sphere about the origin
  float xRadius = fabs(_params[QMC5883L_PARAM_CALIB_X_E].data.f[2] - _params[QMC5883L_PARAM_CALIB_X_E].data.f[0])/2;
  float yRadius = fabs(_params[QMC5883L_PARAM_CALIB_Y_E].data.f[2] - _params[QMC5883L_PARAM_CALIB_Y_E].data.f[0])/2;
  float zRadius = fabs(_params[QMC5883L_PARAM_CALIB_Z_E].data.f[2] - _params[QMC5883L_PARAM_CALIB_Z_E].data.f[0])/2;
  
  // check to see if radii are sufficient to consider this a valid calibration
  float quality = 0;
  if (xRadius > 4) quality++;
  if (yRadius > 4) quality++;
  if (zRadius > 4) quality++;
  _params[QMC5883L_PARAM_VECTOR_E].data.f[3] = quality;

  if (quality == 3) {
    // apply scaling
    _params[QMC5883L_PARAM_VECTOR_E].data.f[0] /= xRadius;
    _params[QMC5883L_PARAM_VECTOR_E].data.f[1] /= yRadius;
    _params[QMC5883L_PARAM_VECTOR_E].data.f[2] /= zRadius;

    // apply rotations

    // pitch
    float pitchAng = -degreesToRadians(_subs[QMC5883L_SUB_PITCH_E].param.data.f[0]);
    float cosPitchAng = cos(pitchAng);
    float sinPitchAng = sin(pitchAng);
    float y1 = _params[QMC5883L_PARAM_VECTOR_E].data.f[1];
    float z1 = _params[QMC5883L_PARAM_VECTOR_E].data.f[2];
    _params[QMC5883L_PARAM_VECTOR_E].data.f[1] = y1 * cosPitchAng - z1 * sinPitchAng;
    _params[QMC5883L_PARAM_VECTOR_E].data.f[2] = y1 * sinPitchAng + z1 * cosPitchAng;

    // roll 
    float rollAng = -degreesToRadians(_subs[QMC5883L_SUB_ROLL_E].param.data.f[0]);
    float cosRollAng = cos(rollAng);
    float sinRollAng = sin(rollAng);
    float x1 = _params[QMC5883L_PARAM_VECTOR_E].data.f[0];
    z1 = _params[QMC5883L_PARAM_VECTOR_E].data.f[2];
    _params[QMC5883L_PARAM_VECTOR_E].data.f[0] = x1 * cosRollAng - z1 * sinRollAng;
    _params[QMC5883L_PARAM_VECTOR_E].data.f[2] = x1 * sinRollAng + z1 * cosRollAng;
  }


  if (_params[QMC5883L_PARAM_MODE_E].data.uint8[0] == QMC5883L_MODE_RESET_CALIBRATION) {

    // reset calibration
    _params[QMC5883L_PARAM_CALIB_X_E].data.f[0] = _params[QMC5883L_PARAM_RAW_E].data.f[0];
    _params[QMC5883L_PARAM_CALIB_X_E].data.f[1] = _params[QMC5883L_PARAM_RAW_E].data.f[0];
    _params[QMC5883L_PARAM_CALIB_X_E].data.f[2] = _params[QMC5883L_PARAM_RAW_E].data.f[0];

    _params[QMC5883L_PARAM_CALIB_Y_E].data.f[0] = _params[QMC5883L_PARAM_RAW_E].data.f[1];
    _params[QMC5883L_PARAM_CALIB_Y_E].data.f[1] = _params[QMC5883L_PARAM_RAW_E].data.f[1];
    _params[QMC5883L_PARAM_CALIB_Y_E].data.f[2] = _params[QMC5883L_PARAM_RAW_E].data.f[1];

    _params[QMC5883L_PARAM_CALIB_Z_E].data.f[0] = _params[QMC5883L_PARAM_RAW_E].data.f[2];
    _params[QMC5883L_PARAM_CALIB_Z_E].data.f[1] = _params[QMC5883L_PARAM_RAW_E].data.f[2];
    _params[QMC5883L_PARAM_CALIB_Z_E].data.f[2] = _params[QMC5883L_PARAM_RAW_E].data.f[2];


    _params[QMC5883L_PARAM_MODE_E].data.uint8[0] = QMC5883L_MODE_ONLINE_CALIBRATION;
  }


  if (_params[QMC5883L_PARAM_MODE_E].data.uint8[0] == QMC5883L_MODE_STORE_CALIBRATION) {
    // write calibration values to EEPROM
    Preferences pref; 
    // use module name as preference namespace
    pref.begin(_mgmtParams[DRONE_MODULE_PARAM_NAME_E].data.c, false);

    // X
    pref.putFloat("xMin", _params[QMC5883L_PARAM_CALIB_X_E].data.f[0]);
    pref.putFloat("xMax", _params[QMC5883L_PARAM_CALIB_X_E].data.f[2]);

    // Y
    pref.putFloat("yMin", _params[QMC5883L_PARAM_CALIB_Y_E].data.f[0]);
    pref.putFloat("yMax", _params[QMC5883L_PARAM_CALIB_Y_E].data.f[2]);

    // Z
    pref.putFloat("zMin", _params[QMC5883L_PARAM_CALIB_Z_E].data.f[0]);
    pref.putFloat("zMax", _params[QMC5883L_PARAM_CALIB_Z_E].data.f[2]);

    pref.end();

    _params[QMC5883L_PARAM_MODE_E].data.uint8[0] = QMC5883L_MODE_FIXED_CALIBRATION;
  }

  float x = _params[QMC5883L_PARAM_VECTOR_E].data.f[0];
  float y = _params[QMC5883L_PARAM_VECTOR_E].data.f[1];
  
  // calculate heading
  float heading = atan2(y, x);

  // rotate by -90 deg to account for sensor mounting orientation with y+ forward
  heading -= PI/2;

  float declinationAngle = _params[QMC5883L_PARAM_DECLINATION_E].data.f[0] * PI / 180.0f; // convert to radians
  heading += declinationAngle;

  // Convert radians to degrees for readability.
  float headingDegrees = (heading * 180.0f / PI);

  // add trim
  headingDegrees += _params[QMC5883L_PARAM_TRIM_E].data.f[0];

  // wrap to 0..360
  headingDegrees = fmod(headingDegrees, 360);
  if (headingDegrees < 0) headingDegrees += 360;

  _params[QMC5883L_PARAM_HEADING_E].data.f[0] = headingDegrees;

  // publish param entries
  publishParamEntries();
}
