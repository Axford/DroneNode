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
struct DEM_NAMESPACE;
struct DEM_INSTRUCTION_COMPILED;
struct DEM_MACRO;
struct DEM_COMMAND;

#define DEM_COMMAND_LENGTH 10  // max command length
#define DEM_NAMESPACE_LENGTH   16
#define DEM_TOKEN_LENGTH  22 // to allow for long GPS coordinates

#define DEM_CALLSTACK_SIZE    64  // 64 x uint32_t = 256 bytes
#define DEM_DATASTACK_SIZE    64  // 64 x uint32_t = 256 bytes

#define DEM_EEPROM_SIZE              1
#define DEM_EEPROM_SUCCESSFUL_BOOT   0

#define DEM_BOOT_FAIL     0  // didnt make it to module setup
#define DEM_BOOT_SUCCESS  1  // made it to module setup

#define DEM_DATATYPE_NONE 255 // dummy dataType for no parameters

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
  int i;  // instruction number
  boolean continuation; // true if we're calling this command for a second time
};

struct DEM_CALLSTACK {
  int8_t p; // stack pointer
  DEM_CALLSTACK_ENTRY stack[DEM_CALLSTACK_SIZE];
};

struct DEM_DATASTACK_ENTRY {
  uint32_t d;
  DEM_INSTRUCTION_COMPILED* owner;  // which instruction put/updated this data value here
};

struct DEM_DATASTACK {
  int8_t p; // stack pointer
  DEM_DATASTACK_ENTRY stack[DEM_DATASTACK_SIZE];
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
  boolean isModuleType; // set to true if this namespace is mapped to a module type
  IvanLinkedList::LinkedList<DEM_COMMAND> *commands;
};

struct DEM_MACRO {
  char name[DRONE_LINK_MSG_MAX_PAYLOAD+1];
  DRONE_LINK_ADDR eventAddr;  // set if this is an event handler
  IvanLinkedList::LinkedList<DEM_INSTRUCTION_COMPILED> *commands;
};

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
  DEM_CALLSTACK _call;
  DEM_DATASTACK _data;

  IvanLinkedList::LinkedList<DEM_MACRO*> _macros;
  IvanLinkedList::LinkedList<DEM_NAMESPACE*> _namespaces;
  IvanLinkedList::LinkedList<DEM_NAMESPACE*> _types; // registered module types

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
  DEM_INSTRUCTION _instruction;  // newly parsed instruction

  unsigned long _maxExecutionTime;
  String _slowestInstruction;

  //IvanLinkedList::LinkedList<DroneLinkChannel*> _channels;
  //DRONE_LINK_NODE_PAGE *_nodePages[DRONE_LINK_NODE_PAGES];

public:
    DroneExecutionManager(DroneSystem* ds, File &logFile);

    uint8_t getBootStatus();
    void setBootStatus(uint8_t v);
    boolean safeMode();  // get safeMode status

    // looks up macro by name, returns null if not found
    DEM_MACRO* getMacro(const char* name);
    // allocates memory, adds to list and returns new macro
    // or returns existing macro address if already created
    DEM_MACRO* createMacro(const char* name);

    DEM_NAMESPACE* getNamespace(const char* name);

    DEM_NAMESPACE* createNamespace(const char* name, uint8_t module, boolean isModuleType);
    DEM_NAMESPACE* createNamespace(const __FlashStringHelper* name, uint8_t module, boolean isModuleType);

    void registerCommand(DEM_NAMESPACE* ns, const char* command, uint8_t dataType, DEMCommandHandler handler);

    DEM_COMMAND getCommand(DEM_NAMESPACE* ns, const char* command);

    void addToAddressQueue(DroneModule* newMod, char* subName, char* address);
    void processAddressQueue();

    void loadConfiguration(const char* filename);
    void saveConfiguration();

    void callStackPush(DEM_CALLSTACK_ENTRY entry);
    void callStackPop();
    DEM_CALLSTACK_ENTRY* callStackPeek(uint8_t offset);

    void dataStackPush(uint32_t d, DEM_INSTRUCTION_COMPILED* owner);
    DEM_DATASTACK_ENTRY* dataStackPop();
    // peek at an item offset down from the top of the stack
    DEM_DATASTACK_ENTRY* dataStackPeek(uint8_t offset);

    void printInstruction(DEM_INSTRUCTION * instruction);

    void parseEnums(DEM_INSTRUCTION * instruction, DEM_ENUM_MAPPING * mappingTable, uint16_t mappings);

    //void compileInstruction(DEM_INSTRUCTION * instruction, DEM_INSTRUCTION_COMPILED * compiled);

    // load a D-Code script from SPIFFS, return true if loaded and parsed ok
    boolean load(const char * filename);

    boolean tokenContainsNumber(char tokenStart);
    boolean compileLine(const char * line, DEM_INSTRUCTION_COMPILED* instr);

    // execute next instruction
    void execute();

    // interrupt whatever we're doing by running a macro
    void runMacro(const char * macroName, boolean calledFromMacro);

    void serveMacroInfo(AsyncWebServerRequest *request);
    void serveCommandInfo(AsyncWebServerRequest *request);
    void serveExecutionInfo(AsyncWebServerRequest *request);


    // core handlers
    boolean core_counter(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_delay(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_do(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_done(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_load(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_module(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_node(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_publish(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_restart(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_run(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_send(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    
    void completeSetup();
    
    boolean core_setup(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_swap(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_until(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);

    DroneModule* instanceModule(char* typeName, uint8_t id);

    boolean mod_constructor(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean mod_param(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean mod_subAddr(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
    boolean core_sub(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation);
};


#endif
