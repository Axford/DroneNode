#ifndef DRONE_EXECUTION_MANAGER_H
#define DRONE_EXECUTION_MANAGER_H

#include "Arduino.h"
#include "LinkedList.h"

#include "DroneLinkMsg.h"
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>

// forward declarations
class DroneModule;
class DroneLinkManager;
class DroneModuleManager;


#define DEM_ENUM_LENGTH   5
#define DEM_TOKEN_LENGTH  16

#define DEM_CALLSTACK_SIZE    64  // 64 x uint32_t = 256 bytes
#define DEM_DATASTACK_SIZE    64  // 64 x uint8_t = 64 bytes

struct DEM_ENUM_MAPPING {
  const char *const str;
  uint8_t value;
};


struct DEM_NAMESPACE {
  char name[DEM_ENUM_LENGTH+1];    // 5 char + \0
  uint8_t module;  // module address
};

union DEM_TOKEN_VALUE {
  float f;
  uint32_t uint32;
  uint8_t uint8;
  char c;
} __packed;

struct DEM_TOKEN {
  char txt[DEM_TOKEN_LENGTH+1]; // raw token characters, max chars + \0
  DEM_TOKEN_VALUE value;  // decoded value
  boolean isEnum;
};

struct DEM_INSTRUCTION {
  uint8_t ns;  // index into namespace array
  char command[2];
  DRONE_LINK_ADDR addr;
  uint8_t dataType;
  uint8_t numTokens;
  DEM_TOKEN tokens[DRONE_LINK_MSG_MAX_PAYLOAD]; // max 16 tokens (e.g. 16x true false enums)
};

struct DEM_INSTRUCTION_COMPILED {
  uint8_t ns;
  uint8_t command;
  DRONE_LINK_MSG msg;
};

struct DEM_MACRO {
  char name[DRONE_LINK_MSG_MAX_PAYLOAD+1];
  DRONE_LINK_ADDR eventAddr;  // set if this is an event handler
  IvanLinkedList::LinkedList<DEM_INSTRUCTION_COMPILED> commands;
};


class DroneExecutionManager
{
protected:
  int8_t _callStackPointer;
  uint32_t _callStack[DEM_CALLSTACK_SIZE];

  int8_t _dataStackPointer;
  uint8_t _dataStack[DEM_DATASTACK_SIZE];

  IvanLinkedList::LinkedList<DEM_MACRO*> _macros;

  DroneLinkManager *_dlm;
  DroneModuleManager *_dmm;
  boolean _scriptLoaded;  // set true by .load()
  //unsigned long _filePos; // where did we last read up to
  const char * _filename;
  File _file;

  uint8_t _channelContext;  // set by CP command
  DEM_INSTRUCTION _instruction;  // newly parsed instruction
  DEM_INSTRUCTION_COMPILED _compiledInstruction;

  //IvanLinkedList::LinkedList<DroneLinkChannel*> _channels;
  //DRONE_LINK_NODE_PAGE *_nodePages[DRONE_LINK_NODE_PAGES];

public:
    DroneExecutionManager(DroneModuleManager *dmm, DroneLinkManager *dlm);

    // looks up macro by name, returns null if not found
    DEM_MACRO* getMacro(const char* name);
    // allocates memory, adds to list and returns new macro
    // or returns existing macro address if already created
    DEM_MACRO* createMacro(const char* name);


    void printInstruction(DEM_INSTRUCTION * instruction);

    void parseEnums(DEM_INSTRUCTION * instruction, DEM_ENUM_MAPPING * mappingTable, uint16_t mappings);

    //void compileInstruction(DEM_INSTRUCTION * instruction, DEM_INSTRUCTION_COMPILED * compiled);

    // load a D-Code script from SPIFFS, return true if loaded and parsed ok
    boolean load(const char * filename);

    boolean tokenContainsNumber(char tokenStart);
    void executeNextLine();

    // process current script
    void loop();

    void serveMacroInfo(AsyncWebServerRequest *request);
};


#endif
