#include "AvoidModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../navMath.h"
#include "AsyncUDP.h"

AvoidModule::AvoidModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(AVOID_STR_AVOID));

   // set default interval to 1000
   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 1000;

   _newPackets[0] = 0;
   _newPackets[1] = 0;

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

   param = &_params[AVOID_PARAM_ETC_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, AVOID_PARAM_ETC);
   setParamName(FPSTR(STRING_ETC), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4 );
   
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
      _newPackets[0]++;

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
          //_params[AVOID_PARAM_VESSEL_E].data.uint32[3] = _sentence._m18.mmsi;

          _newPackets[1]++;
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

  // copy course to adjHeading as default
  float adjHeading = _subs[AVOID_SUB_COURSE_E].param.data.f[0];

  uint8_t inRange =0;
  uint8_t colliding = 0;

  AvoidModuleVessel* avoidV = NULL;
  float avoidHeading = 0;
  float earliestEstimatedTimeToCollision = 24*3600;
  float estimatedTimeToCollision = 24*3600;

  boolean calcCrosstrack;
  boolean calcAvoidHeading;

  AvoidModuleVessel* v;
  for (uint8_t i=0; i<_vessels.size(); i++) {
    v = _vessels.get(i);
    calcCrosstrack = false;
    calcAvoidHeading = false;
    estimatedTimeToCollision = 24*3600;

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
        //avoidHeading = vesselToNode;
        calcCrosstrack = true;
        calcAvoidHeading = true;
        if (_subs[AVOID_SUB_SOG_E].param.data.f[0] > 0)
          estimatedTimeToCollision = d / ( _subs[AVOID_SUB_SOG_E].param.data.f[0] / 1.94384);
      } 
      
      
      if (!calcAvoidHeading) {
        // calc angle between vessel heading vector and vector from vessel to node, if subtended angle is >90 then node is behind vessel
        float vesselToNode = calculateInitialBearingBetweenCoordinates(
          v->location[0],
          v->location[1],
          _subs[AVOID_SUB_LOCATION_E].param.data.f[0],
          _subs[AVOID_SUB_LOCATION_E].param.data.f[1]
        );

        float vesselToNodeToHeading = shortestSignedDistanceBetweenCircularValues(v->courseOverGround, vesselToNode);

        // see if we're in front of the vessel
        if (abs(vesselToNodeToHeading) < 90) {
          // in front
          Serial.println("AIS - in front");

          // calculate cross track info
          calcCrosstrack = true;
        } else {
          //behind
          Serial.println("AIS - behind");
        }
      }

      if (calcCrosstrack) {
        CrosstrackInfo ci;

        // calculate distant point for vessel based on current heading
        GeographicPoint p2;
        p2 = calculateDestinationFromDistanceAndBearing2(
          v->location[0],
          v->location[1],
          10000, 
          v->courseOverGround
        );

        ci = calculateCrosstrackInfo(
          v->location[0],
          v->location[1],
          p2.lon,
          p2.lat,
          _subs[AVOID_SUB_LOCATION_E].param.data.f[0],
          _subs[AVOID_SUB_LOCATION_E].param.data.f[1]
        );

        if (abs(ci.across) < _params[AVOID_PARAM_THRESHOLD_E].data.f[1]) {

          if (v->speedOverGround > 0 && 
              _subs[AVOID_SUB_SOG_E].param.data.f[0] > 0) {

            // calc how long until vessel reaches the crosstrack point
            float timeUntilVesselReachesCrosstrackPoint = max(min(ci.along - 50, d ), 0.0f) / 
                                                          (v->speedOverGround / 1.94384);
            
            // calc time to exit lane at current SOG vs time until collision
            // plan for worst case where we're heading all the way across the lane
            float timeUntilWeCanEscapeTheLane = (_params[AVOID_PARAM_THRESHOLD_E].data.f[1] + abs(ci.across)) /
                                                ( _subs[AVOID_SUB_SOG_E].param.data.f[0] / 1.94384);

            if (timeUntilVesselReachesCrosstrackPoint < timeUntilWeCanEscapeTheLane) {
              estimatedTimeToCollision = timeUntilVesselReachesCrosstrackPoint;
              calcAvoidHeading = true;
            }
          }
        }

        if (calcAvoidHeading && 
            estimatedTimeToCollision < earliestEstimatedTimeToCollision) {
          avoidV = v;
          earliestEstimatedTimeToCollision = estimatedTimeToCollision;
          colliding++;

          // calc course orthogonal to vessel heading, based on crosstrack sign
          // negative crosstrack means node is to starboard of vessel's lane
          avoidHeading = v->courseOverGround;
          if (ci.across < 0) {
            // add 90 degrees
            avoidHeading += 90;
          } else {
            avoidHeading -= 90;
          }
        }
      }

      inRange++;
    }
  }

  uint32_t newVessel[4];
  newVessel[0] = _vessels.size();
  newVessel[1] = inRange;
  newVessel[2] = colliding;

  float newTarget[4];

  if (avoidV) {
    // ensure in range
    avoidHeading = fmod(avoidHeading, 360);
    if (avoidHeading < 0) avoidHeading += 360;
    
    adjHeading = avoidHeading;

    // mmsi
    newVessel[3] = avoidV->mmsi;

    // target info
    newTarget[0] = avoidV->location[0];
    newTarget[1] = avoidV->location[1];
    newTarget[2] = avoidV->courseOverGround;
    newTarget[3] = avoidV->speedOverGround;

  } else {
    newVessel[3] = 0;
    
    newTarget[0] = 0;
    newTarget[1] = 0;
    newTarget[2] = 0;
    newTarget[3] = 0;
  }

  updateAndPublishParam(&_params[AVOID_PARAM_ADJ_HEADING_E], (uint8_t*)&adjHeading, sizeof(adjHeading));
  updateAndPublishParam(&_params[AVOID_PARAM_VESSEL_E], (uint8_t*)&newVessel, sizeof(newVessel));
  updateAndPublishParam(&_params[AVOID_PARAM_TARGET_E], (uint8_t*)&newTarget, sizeof(newTarget));
  updateAndPublishParam(&_params[AVOID_PARAM_ETC_E], (uint8_t*)&earliestEstimatedTimeToCollision, sizeof(earliestEstimatedTimeToCollision));
  updateAndPublishParam(&_params[AVOID_PARAM_PACKETS_E], (uint8_t*)&_newPackets, sizeof(_newPackets));
}


