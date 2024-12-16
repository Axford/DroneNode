#ifndef DRONE_EXECUTION_MANAGER_H
#define DRONE_EXECUTION_MANAGER_H

#include "Arduino.h"
#include "LinkedList.h"

#include "DroneLinkMsg.h"
#include <ESPAsyncWebServer.h>
#include <functional>
#include "FS.h"

// forward declarations
class DroneSystem;
class DroneModule;
class DroneLinkManager;
class DroneModuleManager;


#define DEM_EEPROM_SIZE              1
#define DEM_EEPROM_SUCCESSFUL_BOOT   0

#define DEM_BOOT_FAIL     0  // didnt make it to module setup
#define DEM_BOOT_SUCCESS  1  // made it to module setup

#define DEM_DATATYPE_NONE 255 // dummy dataType for no parameters


struct DEM_ADDRESS {
  uint8_t moduleId;  // what module is subscribing
  void* ps;  // what sub is this address value for 
  char nodeAddress[20];
  char moduleAddress[20];
  char paramAddress[20];
};

static const char DEM_BOOT_FILENAME[] PROGMEM = "/boot.dat";


#define DEM_PARSER_GENERAL 0
#define DEM_PARSER_SECTION_TITLE 1
#define DEM_PARSER_SECTION 2

#define DEM_PARSER_NAME_BUFFER_SIZE 20
#define DEM_PARSER_VALUE_BUFFER_SIZE 200

class DroneExecutionManager
{
protected:
  // queue of addresses that need to be resolved
  IvanLinkedList::LinkedList<DEM_ADDRESS> _addressQueue;

  boolean _safeMode;

  DroneSystem* _ds;
  DroneLinkManager *_dlm;
  DroneModuleManager *_dmm;

  //File _file;  // TODO - can prob remove this
  fs::FS &_fs;
  File &_logFile;
  uint8_t _channelContext;  // set by module commands
  uint8_t _nodeContext;  // set by node command
  boolean _multiLineComment; // set to true when /* encountered
  
  unsigned long _maxExecutionTime;
  String _slowestInstruction;

public:
    DroneExecutionManager(DroneSystem* ds, File &logFile);

    uint8_t getBootStatus();
    void setBootStatus(uint8_t v);
    boolean safeMode();  // get safeMode status

    void addToAddressQueue(DroneModule* newMod, char* subName, char* address);
    void processAddressQueue();

    void loadConfiguration(const char* filename);
    void saveConfiguration();

    void completeSetup();

    DroneModule* instanceModule(char* typeName, uint8_t id);
};


#endif
