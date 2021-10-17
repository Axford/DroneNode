#include "BasicNavModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

BasicNavModule::BasicNavModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem):
  DroneModule ( id, dmm, dlm, dem )
 {
   setTypeName(FPSTR(BASIC_NAV_STR_BASIC_NAV));

   _numParamEntries = BASIC_NAV_PARAM_ENTRIES;
   _params = new DRONE_PARAM_ENTRY[_numParamEntries];

   _target.param = 255;  // junk param
   _location.param = 255;  // junk param

   // defaults
   for (uint8_t i=0; i<_numParamEntries; i++) {
     _params[i].publish = false;
     _params[i].data.f[0] = 0;
     _params[i].data.f[1] = 0;
   }

   _mgmtParams[DRONE_MODULE_PARAM_TYPE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, sizeof(BASIC_NAV_STR_BASIC_NAV));
   strncpy_P(_mgmtParams[DRONE_MODULE_PARAM_TYPE_E].data.c, BASIC_NAV_STR_BASIC_NAV, sizeof(BASIC_NAV_STR_BASIC_NAV));

   _params[BASIC_NAV_PARAM_TARGET_E].param = BASIC_NAV_PARAM_TARGET;
   _params[BASIC_NAV_PARAM_TARGET_E].name = FPSTR(STRING_TARGET);
   _params[BASIC_NAV_PARAM_TARGET_E].nameLen = sizeof(STRING_TARGET);
   _params[BASIC_NAV_PARAM_TARGET_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);

   _params[BASIC_NAV_PARAM_LOCATION_E].param = BASIC_NAV_PARAM_LOCATION;
   _params[BASIC_NAV_PARAM_LOCATION_E].name = FPSTR(STRING_LOCATION);
   _params[BASIC_NAV_PARAM_LOCATION_E].nameLen = sizeof(STRING_LOCATION);
   _params[BASIC_NAV_PARAM_LOCATION_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);

   _params[BASIC_NAV_PARAM_HEADING_E].param = BASIC_NAV_PARAM_HEADING;
   _params[BASIC_NAV_PARAM_HEADING_E].name = FPSTR(STRING_HEADING);
   _params[BASIC_NAV_PARAM_HEADING_E].nameLen = sizeof(STRING_HEADING);
   _params[BASIC_NAV_PARAM_HEADING_E].publish = true;
   _params[BASIC_NAV_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[BASIC_NAV_PARAM_DISTANCE_E].param = BASIC_NAV_PARAM_DISTANCE;
   _params[BASIC_NAV_PARAM_DISTANCE_E].name = FPSTR(STRING_DISTANCE);
   _params[BASIC_NAV_PARAM_DISTANCE_E].nameLen = sizeof(STRING_DISTANCE);
   _params[BASIC_NAV_PARAM_DISTANCE_E].publish = true;
   _params[BASIC_NAV_PARAM_DISTANCE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   update();  // set defaults
}


void BasicNavModule::onParamWrite(DRONE_PARAM_ENTRY *param) {
  DroneModule::onParamWrite(param);

  if (param->param == BASIC_NAV_PARAM_TARGET ||
      param->param == BASIC_NAV_PARAM_LOCATION) {
    update();
  }
}


void BasicNavModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

  _target = DroneLinkMsg::parseAddress( obj[STRING_TARGET] );
  if (_target.param != 255) {
    // make read-only
    _params[BASIC_NAV_PARAM_TARGET_E].paramTypeLength &= ~DRONE_LINK_MSG_WRITABLE;
  }

  _location = DroneLinkMsg::parseAddress( obj[STRING_LOCATION] );
  if (_location.param != 255) {
    // make read-only
    _params[BASIC_NAV_PARAM_LOCATION_E].paramTypeLength &= ~DRONE_LINK_MSG_WRITABLE;
  }
}


void BasicNavModule::handleLinkMessage(DroneLinkMsg *msg) {
  DroneModule::handleLinkMessage(msg);

  if (msg->sameAddress(&_target) &&
      msg->type() == DRONE_LINK_MSG_TYPE_FLOAT) {
    //
    if (memcmp(_params[BASIC_NAV_PARAM_TARGET_E].data.c, msg->_msg.payload.c, msg->length()) != 0) {
      memcpy(_params[BASIC_NAV_PARAM_TARGET_E].data.c, msg->_msg.payload.c, msg->length());
      update();
    }
  }

  if (msg->sameAddress(&_location) &&
      msg->type() == DRONE_LINK_MSG_TYPE_FLOAT) {
    //
    if (memcmp(_params[BASIC_NAV_PARAM_LOCATION_E].data.c, msg->_msg.payload.c, msg->length()) != 0) {
      memcpy(_params[BASIC_NAV_PARAM_LOCATION_E].data.c, msg->_msg.payload.c, msg->length());
      update();
    }
  }
}


void BasicNavModule::setup() {
  DroneModule::setup();

  _dlm->subscribe(&_target, this);
  _dlm->subscribe(&_location, this);
}





void BasicNavModule::update() {
  if (!_setupDone) return;
  // formulae from: https://www.movable-type.co.uk/scripts/latlong.html

  /*
  const R = 6371e3; // metres
  const φ1 = lat1 * Math.PI/180; // φ, λ in radians
  const φ2 = lat2 * Math.PI/180;
  const Δφ = (lat2-lat1) * Math.PI/180;
  const Δλ = (lon2-lon1) * Math.PI/180;
  */

  float lat1 =_params[BASIC_NAV_PARAM_LOCATION_E].data.f[1];
  float lon1 =_params[BASIC_NAV_PARAM_LOCATION_E].data.f[0];

  float lat2 =_params[BASIC_NAV_PARAM_TARGET_E].data.f[1];
  float lon2 =_params[BASIC_NAV_PARAM_TARGET_E].data.f[0];


  float R = 6371e3; // metres
  float lat1r = lat1 * PI/180; // φ, λ in radians
  float lat2r = lat2 * PI/180;
  float lon1r = lon1 * PI/180; // φ, λ in radians
  float lon2r = lon2 * PI/180;
  //float dlat = (lat2-lat1) * PI/180;
  //float dlon = (lon2-lon1) * PI/180;

  /*
  const x = (λ2-λ1) * Math.cos((φ1+φ2)/2);
  const y = (φ2-φ1);
  const d = Math.sqrt(x*x + y*y) * R;
  */
  float x = (lon2r-lon1r) * cos((lat1r+lat2r)/2);
  float y = (lat2r-lat1r);
  float d = sqrt(x*x + y*y) * R;


  y = sin(lon2r-lon1r) * cos(lat2r);
  x = cos(lat1r)*sin(lat2r) -
      sin(lat1r)*cos(lat2r)*cos(lon2r-lon1r);
  float p = atan2(y, x);
  float h = fmod(p*180/PI + 360, 360); // in degrees

  _params[BASIC_NAV_PARAM_HEADING_E].data.f[0] = h;
  _params[BASIC_NAV_PARAM_DISTANCE_E].data.f[0] = d;

  publishParamEntries();
}
