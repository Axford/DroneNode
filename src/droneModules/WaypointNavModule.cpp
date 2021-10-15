#include "WaypointNavModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

WaypointNavModule::WaypointNavModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(WAYPOINT_NAV_STR_WAYPOINT_NAV));

   _loopInterval = 1000;

   _numParamEntries = WAYPOINT_NAV_PARAM_ENTRIES;
   _params = new DRONE_PARAM_ENTRY[_numParamEntries];

   _publishWaypoint = 0;
   _updateNeeded = true;

   _allocatedWaypoints = 0;
   if (_allocatedWaypoints > 0) {
     _waypoints = new DRONE_PARAM_ENTRY[_allocatedWaypoints];
     for (uint8_t i=0; i<_allocatedWaypoints; i++) {
       initWaypoint(i);
     }
   }

   _location.param = 255;  // junk param

   // defaults
   for (uint8_t i=0; i<_numParamEntries; i++) {
     _params[i].publish = false;
     _params[i].data.f[0] = 0;
     _params[i].data.f[1] = 0;
   }

   _params[WAYPOINT_NAV_PARAM_TARGET_E].param = WAYPOINT_NAV_PARAM_TARGET;
   _params[WAYPOINT_NAV_PARAM_TARGET_E].name = FPSTR(STRING_TARGET);
   _params[WAYPOINT_NAV_PARAM_TARGET_E].nameLen = sizeof(STRING_TARGET);
   _params[WAYPOINT_NAV_PARAM_TARGET_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);

   _params[WAYPOINT_NAV_PARAM_LOCATION_E].param = WAYPOINT_NAV_PARAM_LOCATION;
   _params[WAYPOINT_NAV_PARAM_LOCATION_E].name = FPSTR(STRING_LOCATION);
   _params[WAYPOINT_NAV_PARAM_LOCATION_E].nameLen = sizeof(STRING_LOCATION);
   _params[WAYPOINT_NAV_PARAM_LOCATION_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);

   _params[WAYPOINT_NAV_PARAM_HEADING_E].param = WAYPOINT_NAV_PARAM_HEADING;
   _params[WAYPOINT_NAV_PARAM_HEADING_E].name = FPSTR(STRING_HEADING);
   _params[WAYPOINT_NAV_PARAM_HEADING_E].nameLen = sizeof(STRING_HEADING);
   _params[WAYPOINT_NAV_PARAM_HEADING_E].publish = true;
   _params[WAYPOINT_NAV_PARAM_HEADING_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[WAYPOINT_NAV_PARAM_DISTANCE_E].param = WAYPOINT_NAV_PARAM_DISTANCE;
   _params[WAYPOINT_NAV_PARAM_DISTANCE_E].name = FPSTR(STRING_DISTANCE);
   _params[WAYPOINT_NAV_PARAM_DISTANCE_E].nameLen = sizeof(STRING_DISTANCE);
   _params[WAYPOINT_NAV_PARAM_DISTANCE_E].publish = true;
   _params[WAYPOINT_NAV_PARAM_DISTANCE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   _params[WAYPOINT_NAV_PARAM_WAYPOINTS_E].param = WAYPOINT_NAV_PARAM_WAYPOINTS;
   _params[WAYPOINT_NAV_PARAM_WAYPOINTS_E].name = FPSTR(STRING_WAYPOINTS);
   _params[WAYPOINT_NAV_PARAM_WAYPOINTS_E].nameLen = sizeof(STRING_WAYPOINTS);
   _params[WAYPOINT_NAV_PARAM_WAYPOINTS_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[WAYPOINT_NAV_PARAM_WAYPOINTS_E].data.uint8[0] = _allocatedWaypoints;

   _params[WAYPOINT_NAV_PARAM_WAYPOINT_E].param = WAYPOINT_NAV_PARAM_WAYPOINT;
   _params[WAYPOINT_NAV_PARAM_WAYPOINT_E].name = FPSTR(STRING_WAYPOINT);
   _params[WAYPOINT_NAV_PARAM_WAYPOINT_E].nameLen = sizeof(STRING_WAYPOINT);
   _params[WAYPOINT_NAV_PARAM_WAYPOINT_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[WAYPOINT_NAV_PARAM_WAYPOINT_E].data.uint8[0] = 0;

   _params[WAYPOINT_NAV_PARAM_LOOPTO_E].param = WAYPOINT_NAV_PARAM_LOOPTO;
   _params[WAYPOINT_NAV_PARAM_LOOPTO_E].name = FPSTR(STRING_LOOPTO);
   _params[WAYPOINT_NAV_PARAM_LOOPTO_E].nameLen = sizeof(STRING_LOOPTO);
   _params[WAYPOINT_NAV_PARAM_LOOPTO_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[WAYPOINT_NAV_PARAM_LOOPTO_E].data.uint8[0] = 0;
}


void WaypointNavModule::initWaypoint(uint8_t i) {
  _waypoints[i].param = WAYPOINT_NAV_PARAM_WAYPOINTS_START + i;
  _waypoints[i].name = FPSTR(STRING_TARGET);
  _waypoints[i].nameLen = sizeof(STRING_TARGET);
  _waypoints[i].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 12);
  _waypoints[i].publish = true;
  _waypoints[i].data.f[0] = 0;
  _waypoints[i].data.f[1] = 0;
  _waypoints[i].data.f[2] = 1; // 1m target radius
}


void WaypointNavModule::onParamWrite(DRONE_PARAM_ENTRY *param) {
  DroneModule::onParamWrite(param);

  if (param->param == WAYPOINT_NAV_PARAM_WAYPOINT) {
    selectWaypoint(_params[WAYPOINT_NAV_PARAM_WAYPOINT_E].data.uint8[0]);
  }

  if (param->param >= WAYPOINT_NAV_PARAM_WAYPOINTS_START) {
    // see if the active waypoint has been updated
    if ((param->param - WAYPOINT_NAV_PARAM_WAYPOINTS_START) == _params[WAYPOINT_NAV_PARAM_WAYPOINT_E].data.uint8[0]) {
      memcpy(_params[WAYPOINT_NAV_PARAM_TARGET_E].data.c, _waypoints[_params[WAYPOINT_NAV_PARAM_WAYPOINT_E].data.uint8[0]].data.c, 12);
      _updateNeeded =true;
    }
  }
}


void WaypointNavModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

  _location = DroneLinkMsg::parseAddress( obj[STRING_LOCATION] );
  if (_location.param != 255) {
    // make read-only
    _params[WAYPOINT_NAV_PARAM_LOCATION_E].paramTypeLength &= ~DRONE_LINK_MSG_WRITABLE;
  }

  // loopTo
  _params[WAYPOINT_NAV_PARAM_LOOPTO_E].data.uint8[0] = obj[STRING_LOOPTO] | _params[WAYPOINT_NAV_PARAM_LOOPTO_E].data.uint8[0];

  // load waypoints
  if (obj.containsKey(STRING_WAYPOINTS)) {
    Log.noticeln(STRING_WAYPOINTS);
    JsonArray array = obj[STRING_WAYPOINTS].as<JsonArray>();

    // init memory
    _allocatedWaypoints = array.size();
    _waypoints = new DRONE_PARAM_ENTRY[_allocatedWaypoints];
    for (uint8_t i=0; i<_allocatedWaypoints; i++) {
      initWaypoint(i);
    }
    _params[WAYPOINT_NAV_PARAM_WAYPOINTS_E].data.uint8[0] = _allocatedWaypoints;

    uint8_t i = 0;
    for(JsonVariant v : array) {
      // fetch and read contents as an array
      DRONE_PARAM_ENTRY *e = &_waypoints[i];

      JsonArray coords = v.as<JsonArray>();
      if (coords.size() == 3) {
        for (uint8_t j=0; j<3; j++)  e->data.f[j] = coords[j] | e->data.f[j];
      }

      i++;
    }

    selectWaypoint(0);
  }


}


boolean WaypointNavModule::publishParamEntries() {
  boolean res = DroneModule::publishParamEntries();

  if (_publishWaypoint<_allocatedWaypoints) {
    if (_waypoints[_publishWaypoint].publish) {
      res = res && publishParamEntry(&_waypoints[_publishWaypoint]);
    }
    _publishWaypoint++;
    if (_publishWaypoint >= _allocatedWaypoints) _publishWaypoint = 0;
  }
  return res;
}


void WaypointNavModule::handleLinkMessage(DroneLinkMsg *msg) {
  DroneModule::handleLinkMessage(msg);

  if (msg->sameAddress(&_location) &&
      msg->type() == DRONE_LINK_MSG_TYPE_FLOAT) {
    //
    if (memcmp(_params[WAYPOINT_NAV_PARAM_LOCATION_E].data.c, msg->_msg.payload.c, msg->length()) != 0) {
      memcpy(_params[WAYPOINT_NAV_PARAM_LOCATION_E].data.c, msg->_msg.payload.c, msg->length());
    }
    _updateNeeded = true;
  }

  if (msg->param() >= WAYPOINT_NAV_PARAM_WAYPOINTS_START) {
    uint8_t i = msg->param() - WAYPOINT_NAV_PARAM_WAYPOINTS_START;

    if (i < _allocatedWaypoints && msg->param() == _waypoints[i].param) {
      handleParamMessage(msg, &_waypoints[i]);
    }
  }

}


void WaypointNavModule::setup() {
  DroneModule::setup();

  _dlm->subscribe(&_location, this);
}

void WaypointNavModule::loop() {
  DroneModule::loop();

  if (_updateNeeded) {
      update();
  }
}



void WaypointNavModule::update() {

  // formulae from: https://www.movable-type.co.uk/scripts/latlong.html

  /*
  const R = 6371e3; // metres
  const φ1 = lat1 * Math.PI/180; // φ, λ in radians
  const φ2 = lat2 * Math.PI/180;
  const Δφ = (lat2-lat1) * Math.PI/180;
  const Δλ = (lon2-lon1) * Math.PI/180;
  */

  float lat1 =_params[WAYPOINT_NAV_PARAM_LOCATION_E].data.f[1];
  float lon1 =_params[WAYPOINT_NAV_PARAM_LOCATION_E].data.f[0];

  float lat2 =_params[WAYPOINT_NAV_PARAM_TARGET_E].data.f[1];
  float lon2 =_params[WAYPOINT_NAV_PARAM_TARGET_E].data.f[0];


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

  _params[WAYPOINT_NAV_PARAM_HEADING_E].data.f[0] = h;
  _params[WAYPOINT_NAV_PARAM_DISTANCE_E].data.f[0] = d;

  // check to see if we've reached the waypoint
  if (d < _params[WAYPOINT_NAV_PARAM_TARGET_E].data.f[2]) {
    selectWaypoint( _params[WAYPOINT_NAV_PARAM_WAYPOINT_E].data.uint8[0] + 1 );
  }

  _updateNeeded = false;

  publishParamEntries();
}


void WaypointNavModule::selectWaypoint(uint8_t i) {
  if (i >= _allocatedWaypoints) {
    // is a loop point defined?
    if (_params[WAYPOINT_NAV_PARAM_LOOPTO_E].data.uint8[0] > 0) {
      i = _params[WAYPOINT_NAV_PARAM_LOOPTO_E].data.uint8[0]-1;
    }
  }

  if (i < _allocatedWaypoints) {
      _params[WAYPOINT_NAV_PARAM_WAYPOINT_E].data.uint8[0] = i;

      memcpy(_params[WAYPOINT_NAV_PARAM_TARGET_E].data.c, _waypoints[i].data.c, 12);

      _updateNeeded = true;
  }
}
