#include "DroneSystem.h"
#include "DroneExecutionManager.h"
#include "DroneLinkManager.h"
#include "DroneModuleManager.h"
#include "pinConfig.h"
#include "strings.h"
#include "DroneModule.h"

// drone modules
#include "droneModules/AnalogModule.h"
#include "droneModules/AnemometerModule.h"
#include "droneModules/AvoidModule.h"
#include "droneModules/CMPS12Module.h"
#include "droneModules/ControllerModule.h"
#include "droneModules/CylonModule.h"
#include "droneModules/DepthModule.h"
#include "droneModules/DiagnosticModule.h"
#include "droneModules/HMC5883LModule.h"
#include "droneModules/HT16K33Module.h"
#include "droneModules/INA219Module.h"
#include "droneModules/INA3221Module.h"
#include "droneModules/JoystickModule.h"
#include "droneModules/LSM9DS1Module.h"
#include "droneModules/ManagementModule.h"
#include "droneModules/MotorModule.h"
#include "droneModules/MPU6050Module.h"
#include "droneModules/NavModule.h"
#include "droneModules/NeopixelModule.h"
#include "droneModules/NMEAModule.h"
#include "droneModules/NunchuckJoystickModule.h"
#include "droneModules/ODriveModule.h"
#include "droneModules/PanSerialModule.h"
#include "droneModules/PanTiltModule.h"
#include "droneModules/PolarModule.h"
#include "droneModules/ProaModule.h"
#include "droneModules/QMC5883LModule.h"
#include "droneModules/ReceiverModule.h"
#include "droneModules/RFM69TelemetryModule.h"
#include "droneModules/SailorModule.h"
#include "droneModules/SerialTelemetryModule.h"
#include "droneModules/ServoModule.h"
#include "droneModules/SpeedControlModule.h"
#include "droneModules/StatusModule.h"
#include "droneModules/TankSteerModule.h"
#include "droneModules/TurnRateModule.h"
#include "droneModules/UDPTelemetryModule.h"
#include "droneModules/VL53L0XModule.h"
#include "droneModules/WaypointModule.h"
#include "droneModules/WindModule.h"
#include "droneModules/WindFromWingModule.h"

/*
#include "droneModules/TimerModule.h"

#include "droneModules/BME280Module.h"
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
  DEM_ADDRESS addr;
  DRONE_PARAM_SUB* ps = newMod->getSubByName(subName);
  addr.moduleId = newMod->id();

  if (ps == NULL) {
    Log.errorln(F("[DEM.lC] unknown sub name %s"), subName);
    return;
  }
    
  addr.ps = ps;

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

      ps = (DRONE_PARAM_SUB*)addr.ps;

      if (mod) {
        mod->setSubAddr(ps, nodeId, moduleId, paramId);
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

  uint8_t outerState = DEM_PARSER_GENERAL;
  uint8_t inValue = false;  
  boolean inString = false;
  boolean inComment = false;
  boolean addToBuffer = false;

  uint8_t nodeId = 0;
  uint8_t moduleId = 0;
  DroneModule *newMod = NULL;

  uint32_t line = 0;
  char valueBuffer[DEM_PARSER_VALUE_BUFFER_SIZE];  // parsing buffer
  uint8_t vBufLen = 0;  // how many valid chars are in the buffer
  
  char nameBuffer[DEM_PARSER_NAME_BUFFER_SIZE];
  uint8_t nBufLen = 0;


  if (LITTLEFS.exists(F(filename))) {
    File file = LITTLEFS.open(F(filename), FILE_READ);

    if (!file) {
      Serial.print("[w.lW] Error opening: ");
      Serial.println (filename);
    } else {
      
      
      
      while (file.available()) {
        char c = file.read();

        addToBuffer = false;

        if (!inComment) {
          if (inString) {
            // keep accumulating characters until we find a "
            if (c == '"') {
              inString = false;
            } else {
              addToBuffer = true;
            }
          } else {
            addToBuffer = (c != '\t' && 
                          c != ' ' && 
                          c != '\r' && 
                          c != '\n' &&
                          c != '=' && 
                          c != '[' &&
                          c != ']' &&
                          c != '"' &&
                          c != ';');
            if (c == '"') inString = true;
            if (c == ';') inComment = true;
          }

          // accumulate string buffer
          if (addToBuffer) {
            if (inValue) {
              if (vBufLen < DEM_PARSER_VALUE_BUFFER_SIZE-1) {
                valueBuffer[vBufLen] = c;
                vBufLen++;
              }
            } else {
              if (nBufLen < DEM_PARSER_NAME_BUFFER_SIZE-1) {
                nameBuffer[nBufLen] = c;
                nBufLen++;
              }
            }
          }

          if (c == '=' && !inValue) {
            inValue = true;
            vBufLen = 0;
          } else if (c == '[' && nBufLen == 0) {
            outerState = DEM_PARSER_SECTION_TITLE;
            nBufLen = 0;
          }
        }
        

        // if end of line or end of file
        if (c == '\n' || !file.available()) {
          
          // see if we have a valid name=value pair
          if (inValue && vBufLen > 0 && nBufLen > 0) {
            // null terminate
            nameBuffer[nBufLen] = 0;
            valueBuffer[vBufLen] = 0;

            // do the thing
            switch(outerState) {
              case DEM_PARSER_GENERAL:
                  // do we have a node address
                  if (strcmp(nameBuffer, "node") == 0) {
                    nodeId = atoi(valueBuffer);
                    if (nodeId > 0 && nodeId < 255) {
                      // set node address
                      _dmm->node(nodeId);
                      _dlm->node(nodeId);
                      Log.noticeln(F("[DEM.lC] node= %d ..."), nodeId);
                    } else
                      Log.errorln(F("[DEM.lC] invalid node address"));
                  }
                  
                  break;

              case DEM_PARSER_SECTION_TITLE:

                  // parse module id
                   moduleId= atoi(valueBuffer);
                  if (moduleId > 0 && moduleId < 255) {
                    Log.noticeln(F("[DEM.lC] module= %d ..."), moduleId);

                    // see if there is a matching module to instance
                    newMod = instanceModule(nameBuffer, moduleId);

                    yield();
                  } else
                    Log.errorln(F("[DEM.lC] invalid module address"));
                  
                  outerState = DEM_PARSER_SECTION;
                  break;

              case DEM_PARSER_SECTION:

                  if (newMod) {
                    if (strcmp(nameBuffer, "publish")==0) {
                      Log.noticeln("[DEM.lC] publishing: %s", valueBuffer);
                      // parse and publish list of params
                      newMod->publishParamsFromList(valueBuffer);
                    } else if (nameBuffer[0] == '$') {
                      // parse sub
                      addToAddressQueue(newMod, &nameBuffer[1], valueBuffer);

                    } else {
                      // regular param setting
                      Log.noticeln("[DEM.lC] Setting param: %s", nameBuffer);
                      DRONE_PARAM_ENTRY* pe = newMod->getParamEntryByName(nameBuffer);
                      if (pe) {
                        newMod->setParamFromList(pe, valueBuffer);
                      } else {
                        Log.errorln(F("[DEM.lC] Unknown param: %s"), nameBuffer);
                      }
                    }
                    
                  } else 
                    Log.errorln(F("[DEM.lC] no valid module to configure"));
                  break;
            }
          }
          

          nBufLen = 0;
          vBufLen = 0;
          inValue = false;
          line++;
          inString = false;
          inComment = false;
        }
      }
    }

    file.close();
  } else {
    Serial.print("[W.lW] No: ");
    Serial.println(filename);
  }
}


void DroneExecutionManager::saveConfiguration() {

  File f = LITTLEFS.open("/live.ini", "w");
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
}


void DroneExecutionManager::completeSetup() {
  Log.noticeln(F("[.s] Completing module setup"));
  _dmm->setupModules();

  // if we made it to here, can assume no crashes on startup
  setBootStatus(DEM_BOOT_SUCCESS);

  Log.noticeln(F("[.s] Setup complete"));

  // redirect logging to serial

  if (_logFile) _logFile.close();
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
}


/*
  Instance a module by type name
*/

DroneModule* DroneExecutionManager::instanceModule(char* typeName, uint8_t id) {
  Log.noticeln(F("[.c] Instantiating %s as %u"), typeName, id);
  DroneModule *newMod = NULL;

  if (id > 0 && id < 255) {
    if (strcmp_P(typeName, ANALOG_STR_ANALOG) == 0) {
      newMod = new AnalogModule(id, _ds);
    } else if (strcmp_P(typeName, ANEMOMETER_STR_ANEMOMETER) == 0) {
      newMod = new AnemometerModule(id, _ds);
    } else if (strcmp_P(typeName, AVOID_STR_AVOID) == 0) {
      newMod = new AvoidModule(id, _ds);
    } else if (strcmp_P(typeName, CMPS12_STR_CMPS12) == 0) {
      newMod = new CMPS12Module(id, _ds);
    } else if (strcmp_P(typeName, CONTROLLER_STR_CONTROLLER) == 0) {
      newMod = new ControllerModule(id, _ds);
    } else if (strcmp_P(typeName, CYLON_STR_CYLON) == 0) {
      newMod = new CylonModule(id, _ds);
    } else if (strcmp_P(typeName, DEPTH_STR_DEPTH) == 0) {
      newMod = new DepthModule(id, _ds);
    } else if (strcmp_P(typeName, DIAGNOSTIC_STR_DIAGNOSTIC) == 0) {
      newMod = new DiagnosticModule(id, _ds);
    } else if (strcmp_P(typeName, HMC5883L_STR_HMC5883L) == 0) {
      newMod = new HMC5883LModule(id, _ds);
    } else if (strcmp_P(typeName, HT16K33_STR_HT16K33) == 0) {
      newMod = new HT16K33Module(id, _ds);
    } else if (strcmp_P(typeName, INA219_STR_INA219) == 0) {
      newMod = new INA219Module(id, _ds);
    } else if (strcmp_P(typeName, INA3221_STR_INA3221) == 0) {
      newMod = new INA3221Module(id, _ds);
    } else if (strcmp_P(typeName, JOYSTICK_STR_JOYSTICK) == 0) {
      newMod = new JoystickModule(id, _ds);
    } else if (strcmp_P(typeName, LSM9DS1_STR_LSM9DS1) == 0) {
      newMod = new LSM9DS1Module(id, _ds);
    } else if (strcmp_P(typeName, MANAGEMENT_STR_MANAGEMENT) == 0) {
      newMod = new ManagementModule(id, _ds);
    } else if (strcmp_P(typeName, MOTOR_STR_MOTOR) == 0) {
      newMod = new MotorModule(id, _ds);
    } else if (strcmp_P(typeName, MPU6050_STR_MPU6050) == 0) {
      newMod = new MPU6050Module(id, _ds);
    } else if (strcmp_P(typeName, NMEA_STR_NMEA) == 0) {
      newMod = new NMEAModule(id, _ds);
    } else if (strcmp_P(typeName, NEOPIXEL_STR_NEOPIXEL) == 0) {
      newMod = new NeopixelModule(id, _ds);
    } else if (strcmp_P(typeName, NunJOYSTICK_STR_NunJOYSTICK) == 0) {
      newMod = new NunchuckJoystick(id, _ds);
    } else if (strcmp_P(typeName, ODRIVE_STR_ODRIVE) == 0) {
      newMod = new ODriveModule(id, _ds);
    } else if (strcmp_P(typeName, PAN_SERIAL_STR_PAN_SERIAL) == 0) {
      newMod = new PanSerialModule(id, _ds);
    } else if (strcmp_P(typeName, PAN_TILT_STR_PAN_TILT) == 0) {
      newMod = new PanTiltModule(id, _ds);
    } else if (strcmp_P(typeName, POLAR_STR_POLAR) == 0) {
      newMod = new PolarModule(id, _ds);
    } else if (strcmp_P(typeName, PROA_STR_PROA) == 0) {
      newMod = new ProaModule(id, _ds);
    } else if (strcmp_P(typeName, QMC5883L_STR_QMC5883L) == 0) {
      newMod = new QMC5883LModule(id, _ds);
    } else if (strcmp_P(typeName, RECEIVER_STR_RECEIVER) == 0) {
      newMod = new ReceiverModule(id, _ds);
    } else if (strcmp_P(typeName, RFM69_TELEMETRY_STR_RFM69_TELEMETRY) == 0) {
      newMod = new RFM69TelemetryModule(id, _ds);
    } else if (strcmp_P(typeName, SAILOR_STR_SAILOR) == 0) {
      newMod = new SailorModule(id, _ds);
    } else if (strcmp_P(typeName, SERIAL_TELEMETRY_STR_SERIAL_TELEMETRY) == 0) {
      newMod = new SerialTelemetryModule(id, _ds);
    } else if (strcmp_P(typeName, SERVO_STR_SERVO) == 0) {
      newMod = new ServoModule(id, _ds);
    } else if (strcmp_P(typeName, SPEED_CONTROL_STR_SPEED_CONTROL) == 0) {
      newMod = new SpeedControlModule(id, _ds);
    } else if (strcmp_P(typeName, STATUS_STR_STATUS) == 0) {
      newMod = new StatusModule(id, _ds);
    } else if (strcmp_P(typeName, TANK_STEER_STR_TANK_STEER) == 0) {
      newMod = new TankSteerModule(id, _ds);
    } else if (strcmp_P(typeName, TURN_RATE_STR_TURN_RATE) == 0) {
      newMod = new TurnRateModule(id, _ds);
    } else if (strcmp_P(typeName, UDP_TELEMETRY_STR_UDP_TELEMETRY) == 0) {
      newMod = new UDPTelemetryModule(id, _ds);
    } else if (strcmp_P(typeName, VL53L0X_STR_VL53L0X) == 0) {
      newMod = new VL53L0XModule(id, _ds);
    } else if (strcmp_P(typeName, NAV_STR_NAV) == 0) {
      newMod = new NavModule(id, _ds);
    } else if (strcmp_P(typeName, WAYPOINT_STR_WAYPOINT) == 0) {
      newMod = new WaypointModule(id, _ds);
    } else if (strcmp_P(typeName, WIND_STR_WIND) == 0) {
      newMod = new WindModule(id, _ds);
    } else if (strcmp_P(typeName, WIND_FROM_WING_STR_WIND_FROM_WING) == 0) {
      newMod = new WindFromWingModule(id, _ds);
    } else {
      Log.errorln(F("[.c] Unknown type"));
    }

  } else {
    Log.errorln(F("[.c] Invalid id"));
  }
  return newMod;
}

