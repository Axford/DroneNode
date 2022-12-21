/*
@type          Avoid
@inherits      Drone
@description   Sail polar calibration module

@guide >>>

<<<

@config >>>
[Avoid=13]

<<<
*/

#ifndef AVOID_MODULE_H
#define AVOID_MODULE_H

#include "../DroneModule.h"
#include "AsyncUDP.h"
#include "../AIS/AisSentence.h"
#include "LinkedList.h"

/*
Pubs
*/

// @pub 8;f;1;adjHeading;Adjusted heading - either passthrough of <b>Heading</b> or generated to avoid obstacle
#define AVOID_PARAM_ADJ_HEADING    8
#define AVOID_PARAM_ADJ_HEADING_E  0

// @pub 9;f;4;target;Target Lon, Lat, COG and SOG
#define AVOID_PARAM_TARGET         9
#define AVOID_PARAM_TARGET_E       1

// @pub 10;f;2;threshold;Thresholds, 0=range beyond which vessels will be ignored, 1=radius within which to treat as a collision
#define AVOID_PARAM_THRESHOLD      10
#define AVOID_PARAM_THRESHOLD_E    2

// @pub 11;u32;4;vessel;Number of vessels, 0=total, 1=in range, 2=colliding, 3=mmsi of most urgent potential collision
#define AVOID_PARAM_VESSEL         11
#define AVOID_PARAM_VESSEL_E       3

// @pub 12;u32;2;packets;0=Number of NMEA packets recevied, 1=Number of successfully parsed AIS messages
#define AVOID_PARAM_PACKETS        12
#define AVOID_PARAM_PACKETS_E      4

// @pub 13;f;1;ETC;estimated time to collision
#define AVOID_PARAM_ETC            13
#define AVOID_PARAM_ETC_E          5

#define AVOID_PARAM_ENTRIES        6

/*
Subs
*/

// @sub 20;21;f;2;location;Current location from GPS
#define AVOID_SUB_LOCATION           30
#define AVOID_SUB_LOCATION_ADDR      31
#define AVOID_SUB_LOCATION_E         0

// @sub 32;33;f;1;SOG;Speed over ground from GPS
#define AVOID_SUB_SOG                32
#define AVOID_SUB_SOG_ADDR           33
#define AVOID_SUB_SOG_E              1

// @sub 38;39;f;1;heading;Current heading from Compass
#define AVOID_SUB_HEADING            34
#define AVOID_SUB_HEADING_ADDR       35
#define AVOID_SUB_HEADING_E          2

// @sub 34;35;f;1;course;Course from Nav
#define AVOID_SUB_COURSE               36
#define AVOID_SUB_COURSE_ADDR          37
#define AVOID_SUB_COURSE_E             3


#define AVOID_SUBS                   4



struct AvoidModuleVessel {
  uint32_t mmsi;
  float location[2];  // lon, lat
  float courseOverGround;
  float speedOverGround;
};


static const char AVOID_STR_AVOID[] PROGMEM = "Avoid";

class AvoidModule:  public DroneModule {
protected:
  AisSentence _sentence;
  AsyncUDP _udp;
  uint32_t _newPackets[2];
  IvanLinkedList::LinkedList<AvoidModuleVessel*> _vessels;
public:

  AvoidModule(uint8_t id, DroneSystem* ds);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  AvoidModuleVessel* getVesselByMMSI(uint32_t mmsi);

  void setup();
  void update();
  void loop();

};

#endif
