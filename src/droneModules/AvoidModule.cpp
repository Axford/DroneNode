#include "AvoidModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"
#include "AsyncUDP.h"

AvoidModule::AvoidModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(AVOID_STR_AVOID));

   // set default interval to 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   // subs
   initSubs(AVOID_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[AVOID_SUB_LOCATION_E];
   sub->addrParam = AVOID_SUB_LOCATION_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, AVOID_SUB_LOCATION);
   sub->param.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   setParamName(FPSTR(STRING_LOCATION), &sub->param);
   sub->param.data.f[0] =0;
   sub->param.data.f[1] =0;

   sub = &_subs[AVOID_SUB_SOG_E];
   sub->addrParam = AVOID_SUB_SOG_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, AVOID_SUB_SOG);
   setParamName(FPSTR(STRING_SOG), &sub->param);

   sub = &_subs[AVOID_SUB_HEADING_E];
   sub->addrParam = AVOID_SUB_HEADING_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, AVOID_SUB_HEADING);
   setParamName(FPSTR(STRING_HEADING), &sub->param);

   sub = &_subs[AVOID_SUB_COURSE_E];
   sub->addrParam = AVOID_SUB_COURSE_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, AVOID_SUB_COURSE);
   setParamName(FPSTR(STRING_COURSE), &sub->param);


   // pubs
   initParams(AVOID_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[AVOID_PARAM_ADJ_HEADING_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, AVOID_PARAM_ADJ_HEADING);
   setParamName(FPSTR(STRING_ADJ_HEADING), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);

   param = &_params[AVOID_PARAM_TARGET_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_HIGH, AVOID_PARAM_TARGET);
   setParamName(FPSTR(STRING_TARGET), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 16);
   param->data.f[0] = 0;
   param->data.f[1] = 0;
   param->data.f[2] = 0;  // COG
   param->data.f[3] = 0;  // SOG

   param = &_params[AVOID_PARAM_THRESHOLD_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, AVOID_PARAM_THRESHOLD);
   setParamName(FPSTR(STRING_THRESHOLD), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   param->data.f[0] = 10000;  // range beyond which vessels will be ignored in meters
   param->data.f[1] = 50;  // radius within which to treat as a collision

 

   param = &_params[AVOID_PARAM_VESSEL_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, AVOID_PARAM_VESSEL);
   setParamName(FPSTR(STRING_VESSEL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 16);
   param->data.uint32[0] = 0; // total vessels
   param->data.uint32[1] = 0; // in range vessels
   param->data.uint32[2] = 0; // colliding vessels
   param->data.uint32[3] = 0; // mmsi of most urgent colliding vessel


   param = &_params[AVOID_PARAM_PACKETS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_MEDIUM, AVOID_PARAM_PACKETS);
   setParamName(FPSTR(STRING_PACKETS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 8);
   param->data.uint32[0] = 0; // total NMEA packets
   param->data.uint32[1] = 0; // succesfully parsed AIS messages
   
}

DEM_NAMESPACE* AvoidModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(AVOID_STR_AVOID,0,true);
}

void AvoidModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);
  DEMCommandHandler pha = std::bind(&DroneExecutionManager::mod_subAddr, dem, _1, _2, _3, _4);
  dem->registerCommand(ns, STRING_LOCATION, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$location"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_SOG, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$SOG"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_COURSE, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$course"), DRONE_LINK_MSG_TYPE_FLOAT, pha);
  dem->registerCommand(ns, STRING_HEADING, DRONE_LINK_MSG_TYPE_FLOAT, ph);
  dem->registerCommand(ns, PSTR("$heading"), DRONE_LINK_MSG_TYPE_FLOAT, pha);

  dem->registerCommand(ns, STRING_THRESHOLD, DRONE_LINK_MSG_TYPE_FLOAT, ph);
}


AvoidModuleVessel* AvoidModule::getVesselByMMSI(uint32_t mmsi) {
  // get (or create) an entry for requested mmsi
  AvoidModuleVessel* v;
  for (uint8_t i=0; i<_vessels.size(); i++) {
    v = _vessels.get(i);
    if (v->mmsi == mmsi) return v;
  }

  // create new entry
  v = (AvoidModuleVessel*) malloc(sizeof(AvoidModuleVessel));
  v->mmsi = mmsi;
  _vessels.add(v);
  return v;
}


void AvoidModule::setup() {
  DroneModule::setup();

  if (_udp.listen(8008)) {
    
    _udp.onPacket([this](AsyncUDPPacket packet) {
      /*
      Serial.print("UDP Packet Type: ");
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
      Serial.print(", From: ");
      Serial.print(packet.remoteIP());
      Serial.print(":");
      Serial.print(packet.remotePort());
      Serial.print(", To: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Length: ");
      Serial.print(packet.length());
      Serial.print(", Data: ");
      Serial.write(packet.data(), packet.length());
      Serial.println();
      String myString = (const char*)packet.data();

      packet.printf("Got %u bytes of data", packet.length());
      */
      _params[AVOID_PARAM_PACKETS_E].data.uint32[0]++;

      // attempt to parse NMEA sentence
      if (_sentence.parse((char*)packet.data(), packet.length())) {
        // if we recognise the type, then parse payload
        if (_sentence.getAisType() == 18) {

          // update _vessels
          AvoidModuleVessel* v = getVesselByMMSI(_sentence._m18.mmsi);

          if (v) {
            v->courseOverGround = _sentence._m18.courseOverGround;
            v->location[0] = _sentence._m18.lon;
            v->location[1] = _sentence._m18.lat;
            v->speedOverGround = _sentence._m18.speedOverGround;
          }
          // update params
          _params[AVOID_PARAM_VESSEL_E].data.uint32[0] = _vessels.size();
          //_params[AVOID_PARAM_VESSEL_E].data.uint32[3] = _sentence._m18.mmsi;

          _params[AVOID_PARAM_PACKETS_E].data.uint32[1]++;
        }
      }

    });
  } else {
    Serial.println("ERROR: unable to bind to port 8008");
  }
}


void AvoidModule::update() {
  if (!_setupDone) return;

}


void AvoidModule::loop() {
  DroneModule::loop();

  uint8_t inRange =0;
  uint8_t colliding = 0;

  AvoidModuleVessel* avoidV = NULL;

  AvoidModuleVessel* v;
  for (uint8_t i=0; i<_vessels.size(); i++) {
    v = _vessels.get(i);

    float d = calculateDistanceBetweenCoordinates(
      v->location[0],
      v->location[1],
      _subs[AVOID_SUB_LOCATION_E].param.data.f[0],
      _subs[AVOID_SUB_LOCATION_E].param.data.f[1]
    );

    // if within range threshold
    if (d < _params[AVOID_PARAM_THRESHOLD_E].data.f[0]) {
      // see if we're already too close!
      if (d < _params[AVOID_PARAM_THRESHOLD_E].data.f[1]) {
        avoidV = v;
        colliding++;
      } else {
        // see if we're in front of the vessel



      }

      inRange++;
    }
  }

  _params[AVOID_PARAM_VESSEL_E].data.uint32[1] = inRange;
  _params[AVOID_PARAM_VESSEL_E].data.uint32[2] = colliding;

  if (avoidV) {
    // mmsi
    _params[AVOID_PARAM_VESSEL_E].data.uint32[3] = avoidV->mmsi;

    // target info
    _params[AVOID_PARAM_TARGET_E].data.f[0] = avoidV->location[0];
    _params[AVOID_PARAM_TARGET_E].data.f[1] = avoidV->location[1];
    _params[AVOID_PARAM_TARGET_E].data.f[2] = avoidV->courseOverGround;
    _params[AVOID_PARAM_TARGET_E].data.f[3] = avoidV->speedOverGround;

  } else {
    _params[AVOID_PARAM_VESSEL_E].data.uint32[3] = 0;
    _params[AVOID_PARAM_TARGET_E].data.f[0] = 0;
    _params[AVOID_PARAM_TARGET_E].data.f[1] = 0;
    _params[AVOID_PARAM_TARGET_E].data.f[2] = 0;
    _params[AVOID_PARAM_TARGET_E].data.f[3] = 0;
  }

  // copy course to adjHeading for interim
  float adjHeading = _subs[AVOID_SUB_COURSE_E].param.data.f[0];
  updateAndPublishParam(&_params[AVOID_PARAM_ADJ_HEADING_E], (uint8_t*)&adjHeading, sizeof(adjHeading));


  publishParamEntry(&_params[AVOID_PARAM_VESSEL_E]);
  publishParamEntry(&_params[AVOID_PARAM_TARGET_E]);

  publishParamEntry(&_params[AVOID_PARAM_PACKETS_E]);
}


