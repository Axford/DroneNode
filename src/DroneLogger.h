/*

DroneLogger

Manage logging of DroneMesh packets to an SD card.

* Flags to control what message types are captured in the log (DroneLink messages, file transfers, routing updates, etc) - default is everything
* Optional time filtering to lower the capture rate (default is zero, so every message is captured)
* New log files to be auto-sequentially numbered
* A new log is started on reboot and capped to either 1MB or 1hr of data, whichever comes first.  New logs are started automatically

*/
#pragma once

#include "Arduino.h"
#include "DroneMeshMsg.h"
#include "FS.h"

#define CR "\n"
#define LF "\r"
#define NL "\n\r"

class DroneLogger
{
protected:
  boolean _SDAvailable;
  boolean _enabled;
  uint64_t _cardSize;
  uint8_t _cardType;
  File _file;
  uint32_t _lastFlush;

public:
  DroneLogger();

  boolean begin();

  void enable();
  void disable();

  void write(uint8_t * buffer, uint32_t len);
};

extern DroneLogger DroneLog;