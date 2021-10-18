/*

Manages a I2CBASE I2C power monitor

*/
#ifndef NMEA_MODULE_H
#define NMEA_MODULE_H

#include "../DroneModule.h"
#include "MicroNMEA.h"

#define NMEA_PARAM_LOCATION           8
#define NMEA_PARAM_SATELLITES         9
#define NMEA_PARAM_HEADING            10
#define NMEA_PARAM_SPEED              11
#define NMEA_PARAM_HDOP               12
#define NMEA_PARAM_PORT               13
#define NMEA_PARAM_BAUD               14

#define NMEA_PARAM_LOCATION_E         0
#define NMEA_PARAM_SATELLITES_E       1
#define NMEA_PARAM_HEADING_E          2
#define NMEA_PARAM_SPEED_E            3
#define NMEA_PARAM_HDOP_E             4
#define NMEA_PARAM_PORT_E             5
#define NMEA_PARAM_BAUD_E             6

#define NMEA_PARAM_ENTRIES            7

static const char NMEA_STR_NMEA[] PROGMEM = "NMEA";

// class
class NMEAModule:  public DroneModule {
protected:
  //uint8_t _portNum;
  Stream *_port;
  //uint32_t _baud;
  char _buffer[100];
  MicroNMEA *_nmea;
public:

  NMEAModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void onParamWrite(DRONE_PARAM_ENTRY *param);

  void setup();
  void loop();

  void setPort(Stream *port);
};

#endif
