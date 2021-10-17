#ifndef DRONE_EXECUTION_MANAGER_H
#define DRONE_EXECUTION_MANAGER_H

#include "Arduino.h"
#include "LinkedList.h"

#include "DroneLinkMsg.h"
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <functional>

// forward declarations
class DroneModule;
class DroneLinkManager;
class DroneModuleManager;
struct DEM_NAMESPACE;
struct DEM_INSTRUCTION_COMPILED;
struct DEM_MACRO;
struct DEM_COMMAND;

#define DEM_COMMAND_LENGTH 10  // max command length
#define DEM_NAMESPACE_LENGTH   16
#define DEM_TOKEN_LENGTH  16

#define DEM_CALLSTACK_SIZE    64  // 64 x uint32_t = 256 bytes
#define DEM_DATASTACK_SIZE    64  // 64 x uint8_t = 64 bytes


struct DEM_ENUM_MAPPING {
  const char *const str;
  uint8_t value;
};

union DEM_TOKEN_VALUE {
  float f;
  uint32_t uint32;
  uint8_t uint8;
  DRONE_LINK_ADDR addr;
  char c;
} __packed;

struct DEM_TOKEN {
  char txt[DEM_TOKEN_LENGTH+1]; // raw token characters, max chars + \0
  DEM_TOKEN_VALUE value;  // decoded value
  boolean isEnum;
};

struct DEM_INSTRUCTION {
  DEM_NAMESPACE *ns;
  char command[DEM_COMMAND_LENGTH];
  DRONE_LINK_ADDR addr;
  uint8_t dataType;
  uint8_t numTokens;
  DEM_TOKEN tokens[DRONE_LINK_MSG_MAX_PAYLOAD]; // max 16 tokens (e.g. 16x true false enums)
};

struct DEM_CALLSTACK_ENTRY {
  DEM_MACRO* macro;
  uint8_t i;  // instruction number
  boolean continuation; // true if we're calling this command for a second time
};

struct DEM_CALLSTACK {
  int8_t p; // stack pointer
  DEM_CALLSTACK_ENTRY stack[DEM_CALLSTACK_SIZE];
};

struct DEM_DATASTACK {
  int8_t p; // stack pointer
  uint8_t stack[DEM_DATASTACK_SIZE];
};

typedef std::function<boolean(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation)> DEMCommandHandler;

struct DEM_INSTRUCTION_COMPILED {
  DEMCommandHandler handler;
  DEM_NAMESPACE *ns;
  const char * cmd;
  DRONE_LINK_MSG msg;
};

struct DEM_COMMAND {
  const char * str;
  uint8_t dataType;
  DEMCommandHandler handler;
};

struct DEM_NAMESPACE {
  char name[DEM_NAMESPACE_LENGTH+1];    // 5 char + \0
  uint8_t module;  // module address
  IvanLinkedList::LinkedList<DEM_COMMAND> *commands;
};

struct DEM_MACRO {
  char name[DRONE_LINK_MSG_MAX_PAYLOAD+1];
  DRONE_LINK_ADDR eventAddr;  // set if this is an event handler
  IvanLinkedList::LinkedList<DEM_INSTRUCTION_COMPILED> *commands;
};


class DroneExecutionManager
{
protected:
  DEM_CALLSTACK _call;
  DEM_DATASTACK _data;

  IvanLinkedList::LinkedList<DEM_MACRO*> _macros;
  IvanLinkedList::LinkedList<DEM_NAMESPACE*> _namespaces;
  IvanLinkedList::LinkedList<DEM_NAMESPACE*> _types; // registered module types


  DroneLinkManager *_dlm;
  DroneModuleManager *_dmm;
  File _file;  // TODO - can prob remove this

  uint8_t _channelContext;  // set by CP command
  DEM_INSTRUCTION _instruction;  // newly parsed instruction

  //IvanLinkedList::LinkedList<DroneLinkChannel*> _channels;
  //DRONE_LINK_NODE_PAGE *_nodePages[DRONE_LINK_NODE_PAGES];

public:
    DroneExecutionManager(DroneModuleManager *dmm, DroneLinkManager *dlm);

    // looks up macro by name, returns null if not found
    DEM_MACRO* getMacro(const char* name);
    // allocates memory, adds to list and returns new macro
    // or returns existing macro address if already created
    DEM_MACRO* createMacro(const char* name);

    DEM_NAMESPACE* getNamespace(const char* name);

    DEM_NAMESPACE* createNamespace(const char* name, uint8_t module);
    DEM_NAMESPACE* createNamespace(const __FlashStringHelper* name, uint8_t module);

    void registerCommand(DEM_NAMESPACE* ns, const char* command, uint8_t dataType, DEMCommandHandler handler);

    DEM_COMMAND getCommand(DEM_NAMESPACE* ns, const char* command);

    void callStackPush(DEM_CALLSTACK_ENTRY entry);
    void callStackPop();

    void dataStackPush(uint8_t d);
    uint8_t dataStackPop();

    // peek at an item offset down from the top of the stack
    uint8_t dataStackPeek(uint8_t offset);

    void printInstruction(DEM_INSTRUCTION * instruction);

    void parseEnums(DEM_INSTRUCTION * instruction, DEM_ENUM_MAPPING * mappingTable, uint16_t mappings);

    //void compileInstruction(DEM_INSTRUCTION * instruction, DEM_INSTRUCTION_COMPILED * compiled);

    // load a D-Code script from SPIFFS, return true if loaded and parsed ok
    boolean load(const char * filename);

    boolean tokenContainsNumber(char tokenStart);
    boolean compileLine(const char * line, DEM_INSTRUCTION_COMPILED* instr);

    // execute next instruction
    void execute();

    void serveMacroInfo(AsyncWebServerRequest *request);
    void serveCommandInfo(AsyncWebServerRequest *request);

    // command handlers
    boolean core_done(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_load(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_node(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_restart(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_run(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_send(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_setup(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);

    boolean mod_constructor(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean mod_param(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
};


#endif
