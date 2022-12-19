/*

@type          NMEA
@inherits      I2CBase
@description   Manages a serial NMEA GPS device

@config >>>
[NMEA= 7]
  name= "GPS"
  interval= 500
  port= 2
  baud= 9600
  publish =location, satellites, HDOP, followMe
<<<

*/
#ifndef NMEA_MODULE_H
#define NMEA_MODULE_H

#include "../DroneModule.h"
#include "MicroNMEA.h"

// @pub 8;f;2;location;Current GPS location
#define NMEA_PARAM_LOCATION           8

// @pub 9;f;1;satellites;Number of satellites used for fix
#define NMEA_PARAM_SATELLITES         9

// @pub 10;f;1;heading;Current heading over ground
#define NMEA_PARAM_HEADING            10

// @pub 11;f;1;speed;Current speed over ground
#define NMEA_PARAM_SPEED              11

// @pub 12;f;1;HDOP;Current Horizontal Dilution of Precision
#define NMEA_PARAM_HDOP               12

// @pub 13;u8;1;port;Which serial port to use (0,1 or 2)
#define NMEA_PARAM_PORT               13

// @pub 14;u32;1;baud;Baud rate to use, normally 9600
#define NMEA_PARAM_BAUD               14

// @pub 15;f;3;fix;Fix location - to use as basis for differential GPS signal
#define NMEA_PARAM_FIX                15

// @pub 16;f;3;followMe;Location field with a small target radius
#define NMEA_PARAM_FOLLOWME           16

// @pub 17;u32;3;packets;Number of NMEA packets (sentences) received (valid, invalid, unknown)
#define NMEA_PARAM_PACKETS            17

#define NMEA_PARAM_LOCATION_E         0
#define NMEA_PARAM_SATELLITES_E       1
#define NMEA_PARAM_HEADING_E          2
#define NMEA_PARAM_SPEED_E            3
#define NMEA_PARAM_HDOP_E             4
#define NMEA_PARAM_PORT_E             5
#define NMEA_PARAM_BAUD_E             6
#define NMEA_PARAM_FIX_E              7
#define NMEA_PARAM_FOLLOWME_E         8
#define NMEA_PARAM_PACKETS_E          9

#define NMEA_PARAM_ENTRIES            10


// subs

// correction - If fix location is set then this is the difference between live GPS and known location, for other nodes to subscribe to
// if no known location, then this can be subbed to a base station difference param

// @sub 20;21;f;3;correction;If fix location is set then this is the difference between live GPS and known location, for other nodes to subscribe to.  If no fix location, then this can be subbed to a base station correction param.
#define NMEA_SUB_CORRECTION           20
#define NMEA_SUB_CORRECTION_ADDR      21
#define NMEA_SUB_CORRECTION_E         0

#define NMEA_SUBS                     1


#define NMEA_HISTORY_DEPTH            10

static const char NMEA_STR_NMEA[] PROGMEM = "NMEA";

// class
class NMEAModule:  public DroneModule {
protected:
  //uint8_t _portNum;
  Stream *_port;
  //uint32_t _baud;
  char _buffer[100];
  MicroNMEA *_nmea;
  uint8_t _historyCount;
  float _history[NMEA_HISTORY_DEPTH][3];
public:

  NMEAModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  void setup();
  void loop();

  void setPort(Stream *port);

  uint8_t diagnosticDisplays();
  void drawDiagnosticDisplay(SSD1306Wire *display, uint8_t page);
};

#endif
