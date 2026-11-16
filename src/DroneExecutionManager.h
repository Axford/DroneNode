#ifndef DRONE_EXECUTION_MANAGER_H
#define DRONE_EXECUTION_MANAGER_H

#include "Arduino.h"
#include "LinkedList.h"

#include "DroneLinkMsg.h"
#include <ESPAsyncWebServer.h>
#include <functional>
#include "FS.h"
#include "HotLoadConfiguration.h"

// forward declarations
class DroneSystem;
class DroneModule;
class DroneLinkManager;
class DroneModuleManager;
class DigitalModule;
class ProbeModule;
class NMEAModule;
class ServoModule;
class ManagementModule;


#define DEM_EEPROM_SIZE              1
#define DEM_EEPROM_SUCCESSFUL_BOOT   0


#define DEM_DATATYPE_NONE 255 // dummy dataType for no parameters


struct DEM_ADDRESS {
  uint8_t moduleId;  // what module is subscribing
  uint8_t inputId; // subscription address parameter
  char nodeAddress[20];
  char moduleAddress[20];
  char paramAddress[20];
};



#define DEM_PARSER_GENERAL 0
#define DEM_PARSER_SECTION_TITLE 1
#define DEM_PARSER_SECTION 2

#define DEM_PARSER_NAME_BUFFER_SIZE 20
#define DEM_PARSER_VALUE_BUFFER_SIZE 200

class DroneExecutionManager
{
  enum class HotType : uint8_t { StartupOnly, Digital, Probe, NMEA, Servo, Logic };
  struct AppliedSection {
    uint8_t id;
    HotType type;
    bool valid;
    uint8_t digest[32];
  };
  IvanLinkedList::LinkedList<AppliedSection> _appliedSections;
  bool _hotLoadBaselineValid = false;
  char _hotLoadPath[18] = {};
  ManagementModule* _hotLoadRequester = nullptr;

  configuration::HotLoadResult applyHotLoad(const char* path);
  DigitalModule* prepareDigital(uint8_t id);
  ServoModule* prepareServo(uint8_t id);
  NMEAModule* prepareNMEA(uint8_t id);
  ProbeModule* prepareProbe(uint8_t id);
  uint64_t hotLoadPins(DroneModule* module, HotType type);
  void releaseHotLoadPins(DroneModule* module, HotType type);
  bool restoreHotLoadPins(DroneModule* module, HotType type, uint64_t pins);
  bool supportsLogicHotLoad(const char* type);
  DroneModule* prepareLogic(const char* type, uint8_t id);
  struct LiveBinding {
    uint8_t channel;
    char input[20];
    char address[42];
  };
  bool bindLiveInputs(IvanLinkedList::LinkedList<LiveBinding>& bindings,
                      DroneModule** candidates, DroneModule** retiring);
  bool readHotLoadConfiguration(const char* path, IvanLinkedList::LinkedList<AppliedSection>& sections,
                                DroneModule** candidates, configuration::HotLoadResult& result,
                                IvanLinkedList::LinkedList<LiveBinding>* bindings = nullptr);

protected:
  // queue of addresses that need to be resolved
  IvanLinkedList::LinkedList<DEM_ADDRESS> _addressQueue;


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
    bool resumeLiveRoute(const char* channel);
    DroneExecutionManager(DroneSystem* ds, File &logFile);


    bool addToAddressQueue(DroneModule* newMod, char* subName, char* address);
    void processAddressQueue();

    bool loadConfiguration(const char* filename, uint8_t skipped = 0, uint8_t trialNode = 0);
    void saveConfiguration();
    void rememberBootConfiguration(const char* filename);
    void requestHotLoad(ManagementModule* requester, const char* name, size_t length);
    void processHotLoad();
    void runServoConsole(const char* command, Print &output);

    void completeSetup();
    void startRecovery();

    DroneModule* instanceModule(char* typeName, uint8_t id);
    void serveSupportedModules(AsyncWebServerRequest* request) const;
};


#endif
