#include "DroneConsole.h"
#include "ConfigurationLine.h"
#include "droneModules/CloudTelemetryModule.h"
#include "DroneSystem.h"
#include "DroneExecutionManager.h"
#include "DroneLinkManager.h"
#include "DroneModuleManager.h"
#include "pinConfig.h"
#include "strings.h"
#include "DroneModule.h"

// drone modules
#include "droneModules/ADS1015Module.h"
#include "droneModules/AnalogModule.h"
#include "droneModules/AnemometerModule.h"
#include "droneModules/AvoidModule.h"
#include "droneModules/BME280Module.h"
#include "droneModules/CMPS12Module.h"
#include "droneModules/ControllerModule.h"
#include "droneModules/CylonModule.h"
#include "droneModules/DepthModule.h"
#include "droneModules/DiagnosticModule.h"
#include "droneModules/DigitalModule.h"
#include "droneModules/HMC5883LModule.h"
#include "droneModules/HT16K33Module.h"
#include "droneModules/INA219Module.h"
#include "droneModules/INA3221Module.h"
#include "droneModules/InjectModule.h"
#include "droneModules/JoystickModule.h"
#include "droneModules/KiteControllerModule.h"
#include "droneModules/LSM9DS1Module.h"
#include "droneModules/ManagementModule.h"
#include "droneModules/MotorModule.h"
#include "droneModules/MPU6050Module.h"
#include "droneModules/NavModule.h"
#include "droneModules/NeopixelModule.h"
#include "droneModules/NMEAModule.h"
#include "droneModules/ODriveModule.h"
#include "droneModules/PanSerialModule.h"
#include "droneModules/PanTiltModule.h"
#include "droneModules/PolarModule.h"
#include "droneModules/ProbeModule.h"
#include "droneModules/ProaModule.h"
#include "droneModules/QMC5883LModule.h"
#include "droneModules/ReceiverModule.h"
#include "droneModules/RFM69TelemetryModule.h"
#include "droneModules/RotaryModule.h"
#include "droneModules/SailorModule.h"
#include "droneModules/SerialTelemetryModule.h"
#include "droneModules/ServoModule.h"
#include "droneModules/SpeedControlModule.h"
#include "droneModules/StatusModule.h"
#include "droneModules/TankSteerModule.h"
#include "droneModules/TurnRateModule.h"
#include "droneModules/UDPTelemetryModule.h"
#include "droneModules/VL53L0XModule.h"
#include "droneModules/WaterSensorModule.h"
#include "droneModules/WaypointModule.h"
#include "droneModules/WindModule.h"
#include "droneModules/WindFromWingModule.h"

/*
#include "droneModules/TimerModule.h"
#include "droneModules/BasicNavModule.h"

#include "droneModules/OLEDModule.h"



*/


DroneExecutionManager::DroneExecutionManager(DroneSystem* ds, File &logFile):
  _ds(ds),
  _fs(LITTLEFS),  // TODO - replace with dfs reference
  _logFile(logFile) {
  _dmm = _ds->dmm;
  _dlm = _ds->dlm;

  // read last boot status from EEPROM
  //_safeMode
  _safeMode = (getBootStatus() != DEM_BOOT_SUCCESS);
  // disable safeMode - cos its a pain in the ass!
  //_safeMode = false;

  Log.noticeln("[DEM.DEM] SafeMode %u", (_safeMode ? 1: 0));

  // now clear the _safeMode value ready for this boot attempt
  setBootStatus(DEM_BOOT_FAIL);

  //_safeMode = false;

  _channelContext = 0;
  _nodeContext = 0;
  _multiLineComment = false;

  _maxExecutionTime = 0;
  _slowestInstruction = "?";
}


uint8_t DroneExecutionManager::getBootStatus() {
  uint8_t v = DEM_BOOT_FAIL;
  if(_fs.exists(DEM_BOOT_FILENAME)) {
    File f = _fs.open(DEM_BOOT_FILENAME, FILE_READ);
    if (f && f.available()) {
      v = f.read();
      Log.noticeln(F("[DEM.gBS] boot flag: %u"), v);
    }
    f.close();
  } else {
    Log.errorln(F("[DEM.gBS] boot.dat does not exist"));
  }
  return v;
}


void DroneExecutionManager::setBootStatus(uint8_t v) {
  File f = _fs.open(DEM_BOOT_FILENAME, FILE_WRITE);
  f.write(v);
  f.close();
}

boolean DroneExecutionManager::safeMode() {
  return _safeMode;
}


void DroneExecutionManager::addToAddressQueue(DroneModule* newMod, char* subName, char* address) {
  // add to address parsing queue
  DEM_ADDRESS addr{};
  DRONE_PARAM_SUB* ps = newMod->getSubByName(subName);
  addr.moduleId = newMod->id();

  if (ps == NULL) {
    Log.errorln(F("[DEM.lC] unknown sub name %s"), subName);
    return;
  }
    
  addr.inputId = ps->addrParam;

  // parse elements of address
  char buffer[20];
  uint8_t bufLen = 0;
  uint8_t part = 0;
  char c;
  uint8_t i=0;

  do {
    c = address[i];
    if (c == ' ' || c == '\t') {

    } else if (c == '>') {
      if (bufLen > 0) {
        memcpy(addr.nodeAddress, buffer, bufLen);
        addr.nodeAddress[bufLen] = 0;
      }
      bufLen = 0;
      part = 1;
    } else if (c == '.') {
      if (bufLen > 0) {
        memcpy(addr.moduleAddress, buffer, bufLen);
        addr.moduleAddress[bufLen] = 0;
      }
      bufLen = 0;
      part = 2;

    }  else if (c == 0) {
      if (bufLen > 0) {
        memcpy(addr.paramAddress, buffer, bufLen);
        addr.paramAddress[bufLen] = 0;
      }
      bufLen = 0;
      part = 2;

    } else {
      if (bufLen < 19) {
        buffer[bufLen] = c;
        bufLen++;
      }
    }

    i++;
  } while (c > 0);

  Log.noticeln("[DEM.aTAQ] Adding addr to queue: %s,%s,%s", addr.nodeAddress, addr.moduleAddress, addr.paramAddress);
  _addressQueue.add(addr);
}


void DroneExecutionManager::processAddressQueue() {
  if (_addressQueue.size() == 0) return;

  DEM_ADDRESS addr;
  boolean resolved = false;
  uint8_t nodeId = 0;
  uint8_t moduleId = 0;
  uint8_t paramId = 0;
  
  DRONE_PARAM_SUB* ps;

  uint8_t i=0;
  while (i < _addressQueue.size()) {

    addr = _addressQueue.get(i);

    nodeId = 0;
    moduleId = 0;
    paramId = 0;

    resolved = false;

    // try to resolve the node
    if (addr.nodeAddress[0]== '@') {
      nodeId = _dlm->node();
      resolved = true;
    } else {
      // try to convert to an int
      nodeId = atoi(addr.nodeAddress);
      if (nodeId > 0) {
        resolved = true;
      } else {
        // lookup in mesh routing table
        nodeId = _dlm->getNodeByName(addr.nodeAddress);
        if (nodeId > 0) resolved = true;
      }
    }

    if (resolved) {
      // now lets try to resolve the module name
      // only works for local modules
    
      // try to convert to an int
      moduleId = atoi(addr.moduleAddress);
      if (moduleId > 0) { 
        resolved = true;
      } else {
        if (nodeId == _dlm->node()) {
          DroneModule* mod = _dmm->getModuleByName(addr.moduleAddress);
          if (mod) {
            resolved =true;
            moduleId = mod->id();
          } else {
            resolved =false;
          }
        } else {
          resolved =false;
        }
      }
    }

    if (resolved) {
      // finally lets try to resolve the param id
      paramId = atoi(addr.paramAddress);

      if (paramId ==0 && nodeId == _dlm->node()) {
        // if its a local address we can try and resolve names
        DroneModule* mod = _dmm->getModuleById(moduleId);
        if (mod) {
          paramId = mod->getParamIdByName(addr.paramAddress);
        }
      }

      resolved = (paramId > 0);
    }

    if (resolved) {
      // set sub address value
      DroneModule* mod = _dmm->getModuleById(addr.moduleId);

      if (mod) {
        for (uint8_t index = 0; (ps = mod->subscription(index)); ++index) {
          if (ps->addrParam != addr.inputId) continue;
          mod->setSubAddr(ps, nodeId, moduleId, paramId);
          break;
        }
        Log.noticeln("[DEM.pAQ] Resolved: %s>%s.%s to %u>%d.%u", addr.nodeAddress, addr.moduleAddress, addr.paramAddress, nodeId, moduleId, paramId);
      } else {
        Log.errorln("[DEM.pAQ] Unable to resolve sub module %u", addr.moduleId);      }

      _addressQueue.remove(i);
    } else {
      i++;
    }
  }
}


void DroneExecutionManager::loadConfiguration(const char* filename) {
  File file=LITTLEFS.open(filename,FILE_READ);
  if(!file) { Log.errorln("[DEM] Unable to open configuration"); return; }
  DroneModule* module=nullptr;bool inSection=false;
  char raw[2049]; size_t used=0; bool overflow=false; uint32_t lineNumber=0;
  auto applyLine=[&]() {
    ++lineNumber; raw[used]=0;
    configuration::Line line;
    bool valid=line.parse(raw,false);
    if(module && module->isConfigurationString(line.name)) valid=line.parse(raw,true);
    if(overflow||!valid) {
      Log.errorln("[DEM] Invalid configuration at line %u",lineNumber);
      if(module) module->configurationError();
      return;
    }
    if(line.empty) return;
    if(line.section) {
      inSection=true;
      unsigned id=0;
      if(!telemetry::decimal(line.value,254,id)) {
        Log.errorln("[DEM] Invalid module address at line %u",lineNumber);module=nullptr;return;
      }
      module=instanceModule(line.name,id);
      return;
    }
    if(!module) {
      if(!inSection&&!strcmp(line.name,"node")) {
        unsigned id=0;
        if(telemetry::decimal(line.value,254,id)) { _dmm->node(id);_dlm->node(id); }
        else Log.errorln("[DEM] Invalid node address at line %u",lineNumber);
      }
      return;
    }
    // Legacy token scanners use uint8_t indexes; only dedicated string hooks accept long values.
    if(!module->isConfigurationString(line.name) && strlen(line.value)>=DEM_PARSER_VALUE_BUFFER_SIZE) {
      Log.errorln("[DEM] Legacy setting too long at line %u",lineNumber);module->configurationError();return;
    }
    if(!strcmp(line.name,"publish")) module->publishParamsFromList(line.value);
    else if(line.name[0]=='$') addToAddressQueue(module,line.name+1,line.value);
    else {
      auto result=module->configure(line.name,line.value);
      if(result==telemetry::Setting::Invalid) {
        Log.errorln("[DEM] Invalid module configuration at line %u",lineNumber);module->configurationError();
      } else if(result==telemetry::Setting::Unhandled) {
        auto param=module->getParamEntryByName(line.name);
        if(param) module->setParamFromList(param,line.value);
        else Log.errorln("[DEM] Unknown setting at line %u",lineNumber);
      }
    }
  };
  while(file.available()) {
    char c=file.read();
    if(c=='\n') {applyLine();used=0;overflow=false;yield();}
    else if(used<sizeof(raw)-1) raw[used++]=c;
    else overflow=true;
  }
  if(used||overflow) applyLine();
  file.close();
}


void DroneExecutionManager::saveConfiguration() {

  File f = LITTLEFS.open("/config.ini", "w");
  if (f) {
    // save node id
    f.print("node = ");
    f.println(_dmm->node());

    f.println("");

    // for each module
    for (uint8_t i=0; i<_dmm->moduleCount(); i++) {
      DroneModule *m = _dmm->getModuleByIndex(i);
      if (m) {
        m->saveConfiguration(&f);

        f.println("");
      }
    }

    f.close();
    
  }

  _ds->dfs.enumerate();
}


void DroneExecutionManager::completeSetup() {
  Log.noticeln(F("[.s] Completing module setup"));
  _dmm->setupModules();

  // if we made it to here, can assume no crashes on startup
  setBootStatus(DEM_BOOT_SUCCESS);

  Log.noticeln(F("[.s] Setup complete"));

  // redirect logging to serial

  if (_logFile) _logFile.close();
  Log.begin(LOG_LEVEL_VERBOSE, &DroneConsole);
}


/*
  Instance a module by type name
*/

DigitalModule* DroneExecutionManager::prepareDigital(uint8_t id) {
  return new (std::nothrow) DigitalModule(id, _ds, DroneModule::Registration::Detached);
}

ProbeModule* DroneExecutionManager::prepareProbe(uint8_t id) {
  return new (std::nothrow) ProbeModule(id, _ds, DroneModule::Registration::Detached);
}

NMEAModule* DroneExecutionManager::prepareNMEA(uint8_t id) {
  return new (std::nothrow) NMEAModule(id, _ds, DroneModule::Registration::Detached);
}

ServoModule* DroneExecutionManager::prepareServo(uint8_t id) {
  return new (std::nothrow) ServoModule(id, _ds, DroneModule::Registration::Detached);
}

namespace {

template <typename Module>
DroneModule* createModule(uint8_t id, DroneSystem* system) {
  return new Module(id, system);
}

using ModuleCreator = DroneModule* (*)(uint8_t, DroneSystem*);
struct ModuleType {
  const char* name;
  ModuleCreator create;
  ModuleCreator prepare;
  constexpr ModuleType(const char* name, ModuleCreator create, ModuleCreator prepare = nullptr)
      : name(name), create(create), prepare(prepare) {}
};

template <typename Module>
DroneModule* prepareLogicModule(uint8_t id, DroneSystem* system) {
  auto* module = new (std::nothrow) Module(id, system, DroneModule::Registration::Detached);
  if (module && !module->hotLoadReady()) {
    delete module;
    return nullptr;
  }
  return module;
}

// The startup factory and capability endpoint share this immutable registry.
const ModuleType moduleTypes[] = {
  {"CloudTelemetry", createModule<CloudTelemetryModule>},
  {ADS1015_STR_ADS1015, createModule<ADS1015Module>},
  {ANALOG_STR_ANALOG, createModule<AnalogModule>},
  {ANEMOMETER_STR_ANEMOMETER, createModule<AnemometerModule>},
  {AVOID_STR_AVOID, createModule<AvoidModule>},
  {BME280_STR_BME280, createModule<BME280Module>},
  {CMPS12_STR_CMPS12, createModule<CMPS12Module>},
  {CONTROLLER_STR_CONTROLLER, createModule<ControllerModule>},
  {CYLON_STR_CYLON, createModule<CylonModule>},
  {DEPTH_STR_DEPTH, createModule<DepthModule>},
  {"Probe", createModule<ProbeModule>},
  {DIAGNOSTIC_STR_DIAGNOSTIC, createModule<DiagnosticModule>},
  {DIGITAL_STR_DIGITAL, createModule<DigitalModule>},
  {HMC5883L_STR_HMC5883L, createModule<HMC5883LModule>},
  {HT16K33_STR_HT16K33, createModule<HT16K33Module>},
  {INA219_STR_INA219, createModule<INA219Module>},
  {INA3221_STR_INA3221, createModule<INA3221Module>},
  {INJECT_STR_INJECT, createModule<InjectModule>},
  {JOYSTICK_STR_JOYSTICK, createModule<JoystickModule>},
  {KITE_CONTROLLER_STR_KITE_CONTROLLER, createModule<KiteControllerModule>},
  {LSM9DS1_STR_LSM9DS1, createModule<LSM9DS1Module>},
  {MANAGEMENT_STR_MANAGEMENT, createModule<ManagementModule>},
  {MOTOR_STR_MOTOR, createModule<MotorModule>},
  {MPU6050_STR_MPU6050, createModule<MPU6050Module>},
  {NMEA_STR_NMEA, createModule<NMEAModule>},
  {NEOPIXEL_STR_NEOPIXEL, createModule<NeopixelModule>},
  {ODRIVE_STR_ODRIVE, createModule<ODriveModule>},
  {PAN_SERIAL_STR_PAN_SERIAL, createModule<PanSerialModule>},
  {PAN_TILT_STR_PAN_TILT, createModule<PanTiltModule>},
  {POLAR_STR_POLAR, createModule<PolarModule>},
  {PROA_STR_PROA, createModule<ProaModule>},
  {QMC5883L_STR_QMC5883L, createModule<QMC5883LModule>},
  {RECEIVER_STR_RECEIVER, createModule<ReceiverModule>},
  {RFM69_TELEMETRY_STR_RFM69_TELEMETRY, createModule<RFM69TelemetryModule>},
  {ROTARY_STR_ROTARY, createModule<RotaryModule>},
  {SAILOR_STR_SAILOR, createModule<SailorModule>},
  {SERIAL_TELEMETRY_STR_SERIAL_TELEMETRY, createModule<SerialTelemetryModule>},
  {SERVO_STR_SERVO, createModule<ServoModule>},
  {SPEED_CONTROL_STR_SPEED_CONTROL, createModule<SpeedControlModule>},
  {STATUS_STR_STATUS, createModule<StatusModule>, prepareLogicModule<StatusModule>},
  {TANK_STEER_STR_TANK_STEER, createModule<TankSteerModule>},
  {TURN_RATE_STR_TURN_RATE, createModule<TurnRateModule>},
  {UDP_TELEMETRY_STR_UDP_TELEMETRY, createModule<UDPTelemetryModule>},
  {VL53L0X_STR_VL53L0X, createModule<VL53L0XModule>},
  {NAV_STR_NAV, createModule<NavModule>},
  {WATER_SENSOR_STR_WATER_SENSOR, createModule<WaterSensorModule>},
  {WAYPOINT_STR_WAYPOINT, createModule<WaypointModule>},
  {WIND_STR_WIND, createModule<WindModule>},
  {WIND_FROM_WING_STR_WIND_FROM_WING, createModule<WindFromWingModule>},
};

}  // namespace

DroneModule* DroneExecutionManager::instanceModule(char* typeName, uint8_t id) {
  Log.noticeln(F("[.c] Instantiating %s as %u"), typeName, id);
  if (id == 0 || id == 255) {
    Log.errorln(F("[.c] Invalid id"));
    return nullptr;
  }

  for (const auto& type : moduleTypes) {
    if (strcmp_P(typeName, type.name) == 0) return type.create(id, _ds);
  }
  Log.errorln(F("[.c] Unknown type"));
  return nullptr;
}

bool DroneExecutionManager::supportsLogicHotLoad(const char* name) {
  for (const auto& type : moduleTypes)
    if (type.prepare && strcmp_P(name, type.name) == 0) return true;
  return false;
}

DroneModule* DroneExecutionManager::prepareLogic(const char* name, uint8_t id) {
  for (const auto& type : moduleTypes) {
    if (type.prepare && strcmp_P(name, type.name) == 0)
      return type.prepare(id, _ds);
  }
  return nullptr;
}

void DroneExecutionManager::serveSupportedModules(AsyncWebServerRequest* request) const {
  auto* response = request->beginResponseStream("application/json", 1024);
  if (!response) {
    request->send(503, "text/plain", "Module capability response unavailable");
    return;
  }
  response->addHeader("Cache-Control", "no-store");
  response->printf("{\"schemaVersion\":1,\"node\":%u,\"build\":\"%s\",\"types\":[", _dlm->node(), GIT_COMMIT);
  const char* separator = "";
  for (const auto& type : moduleTypes) {
    response->printf("%s\"%s\"", separator, type.name);
    separator = ",";
  }
  response->print("]}");
  request->send(response);
}
