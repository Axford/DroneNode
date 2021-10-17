#include "DroneExecutionManager.h"
#include "DroneLinkManager.h"
#include "DroneModuleManager.h"
#include "pinConfig.h"
#include "strings.h"
#include "DroneModule.h"
#include <EEPROM.h>

// drone modules
#include "droneModules/ManagementModule.h"
#include "droneModules/MotorModule.h"
/*
#include "droneModules/TimerModule.h"
#include "droneModules/ServoModule.h"
#include "droneModules/TelemetryModule.h"
#include "droneModules/UDPTelemetryModule.h"
#include "droneModules/BME280Module.h"
#include "droneModules/INA219Module.h"
#include "droneModules/HMC5883LModule.h"
#include "droneModules/NMEAModule.h"
#include "droneModules/MPU6050Module.h"
#include "droneModules/TankSteerModule.h"
#include "droneModules/BasicNavModule.h"
#include "droneModules/WaypointNavModule.h"
#include "droneModules/TurnRateModule.h"
#include "droneModules/RFM69TelemetryModule.h"
#include "droneModules/JoystickModule.h"
#include "droneModules/OLEDModule.h"
#include "droneModules/ControllerModule.h"
#include "droneModules/NunchuckJoystickModule.h"
#include "droneModules/NeopixelModule.h"
*/


DEM_ENUM_MAPPING DEM_ENUM_TABLE[] PROGMEM = {
  // boolean
  { STRING_FALSE, 0 },
  { STRING_TRUE, 1 },

  // pins
  { PSTR("OUT0_0"), PIN_OUT0_0},
  { PSTR("OUT0_1"), PIN_OUT0_1 },
  { PSTR("OUT1_0"), PIN_OUT1_0 },
  { PSTR("OUT1_1"), PIN_OUT1_1 },
  { PSTR("OUT2_0"), PIN_OUT2_0 },
  { PSTR("OUT2_1"), PIN_OUT2_1 },
  { PSTR("DAC0_0"), PIN_DAC0_0 },
  { PSTR("DAC0_1"), PIN_DAC0_1 },
  { PSTR("IN0_0"), PIN_IN0_0 },
  { PSTR("IN0_1"), PIN_IN0_1 }
};


// used by DEM to parse core param names
DEM_ENUM_MAPPING DRONE_PARAM_TABLE[] PROGMEM = {
  { STRING_STATUS, DRONE_MODULE_PARAM_STATUS_E },
  { STRING_NAME, DRONE_MODULE_PARAM_NAME_E },
  { STRING_ERROR, DRONE_MODULE_PARAM_ERROR_E },
  { STRING_RESETCOUNT, DRONE_MODULE_PARAM_RESETCOUNT_E },
  { STRING_TYPE, DRONE_MODULE_PARAM_TYPE_E },
};



DroneExecutionManager::DroneExecutionManager(DroneModuleManager *dmm, DroneLinkManager *dlm) {
  _dmm = dmm;
  _dlm = dlm;
  _call.p = -1;
  _data.p = -1;

  // read last boot status from EEPROM
  //_safeMode
  EEPROM.begin(DEM_EEPROM_SIZE);
  _safeMode = (EEPROM.read(DEM_EEPROM_SUCCESSFUL_BOOT) != DEM_BOOT_SUCCESS);

  // now clear the _safeMode value ready for this boot attempt
  EEPROM.write(DEM_EEPROM_SUCCESSFUL_BOOT, DEM_BOOT_FAIL);
  EEPROM.commit();

  _instruction.ns = 0;
  _instruction.command[0] = '_';
  _instruction.command[1] = '_';
  _instruction.command[2] = 0;
  _instruction.addr.node = dlm->node();
  _instruction.addr.channel = 0;
  _instruction.addr.param = 0;
  _instruction.dataType = DRONE_LINK_MSG_TYPE_UINT8_T;
  _instruction.numTokens = 0;

  _channelContext = 0;

  // register core commands
  DEM_NAMESPACE *core = createNamespace(PSTR("core"), 0, false);

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  registerCommand(core, PSTR("done"), DRONE_LINK_MSG_TYPE_UINT8_T, std::bind(&DroneExecutionManager::core_done, this, _1, _2, _3, _4));
  registerCommand(core, PSTR("load"), DRONE_LINK_MSG_TYPE_CHAR, std::bind(&DroneExecutionManager::core_load, this, _1, _2, _3, _4));
  registerCommand(core, PSTR("node"), DRONE_LINK_MSG_TYPE_UINT8_T, std::bind(&DroneExecutionManager::core_node, this, _1, _2, _3, _4));
  registerCommand(core, PSTR("publish"), DRONE_LINK_MSG_TYPE_UINT8_T, std::bind(&DroneExecutionManager::core_publish, this, _1, _2, _3, _4));
  registerCommand(core, PSTR("restart"),DRONE_LINK_MSG_TYPE_UINT8_T, std::bind(&DroneExecutionManager::core_restart, this, _1, _2, _3, _4));
  registerCommand(core, PSTR("run"),DRONE_LINK_MSG_TYPE_CHAR, std::bind(&DroneExecutionManager::core_run, this, _1, _2, _3, _4));
  registerCommand(core, PSTR("send"), 255, std::bind(&DroneExecutionManager::core_send, this, _1, _2, _3, _4));
  registerCommand(core, PSTR("setup"), DRONE_LINK_MSG_TYPE_UINT8_T, std::bind(&DroneExecutionManager::core_setup, this, _1, _2, _3, _4));


  // register modules
  DEM_NAMESPACE* ns;

  // register namespaces
  //ns = DroneModule::registerNamespace(this);

  ns = ManagementModule::registerNamespace(this);
  ManagementModule::registerParams(ns, this);

  ns = MotorModule::registerNamespace(this);
  MotorModule::registerParams(ns, this);

  // register constructors and mgmtParams for all module namespaces
  for (uint8_t i=0; i<_namespaces.size(); i++) {
    ns = _namespaces.get(i);
    if (ns->isModuleType) {
      DroneModule::registerConstructor(ns, this);
      DroneModule::registerMgmtParams(ns, this);
    }
  }


  /*
  DEMCommandHandler ch = std::bind(&DroneExecutionManager::mod_constructor, this, _1, _2, _3, _4);
  DEM_NAMESPACE *nsManagement = createNamespace(MANAGEMENT_STR_MANAGEMENT,0);
  registerCommand(nsManagement, STR_NEW, DRONE_LINK_MSG_TYPE_UINT8_T, ch);

  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, this, _1, _2, _3, _4);
  registerCommand(nsManagement, STRING_STATUS, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  registerCommand(nsManagement, STRING_NAME, DRONE_LINK_MSG_TYPE_CHAR, ph);
  */

}



// looks up macro by name, returns null if not found
DEM_MACRO* DroneExecutionManager::getMacro(const char* name) {
  DEM_MACRO* m;
  for(int i = 0; i < _macros.size(); i++){
    m = _macros.get(i);
    if (strcmp(m->name, name) == 0) return m;
  }
  return NULL;
}


// allocates memory, adds to list and returns new macro
// or returns existing macro address if already created
DEM_MACRO* DroneExecutionManager::createMacro(const char* name) {
  DEM_MACRO* res = getMacro(name);

  if (res == NULL) {
    res =  (DEM_MACRO*)malloc(sizeof(DEM_MACRO));
    strcpy(res->name, name);
    res->eventAddr.node = 0;
    res->eventAddr.channel = 0;
    res->eventAddr.param = 0;
    res->commands = new IvanLinkedList::LinkedList<DEM_INSTRUCTION_COMPILED>();

    _macros.add(res);
  }

  return res;
}


DEM_NAMESPACE* DroneExecutionManager::getNamespace(const char* name) {
  DEM_NAMESPACE* n;
  for(int i = 0; i < _namespaces.size(); i++){
    n = _namespaces.get(i);
    if (strcmp(n->name, name) == 0) return n;
  }
  return NULL;
}

DEM_NAMESPACE* DroneExecutionManager::createNamespace(const char* name, uint8_t module, boolean isModuleType) {
  DEM_NAMESPACE* res = getNamespace(name);

  if (res == NULL) {
    res =  (DEM_NAMESPACE*)malloc(sizeof(DEM_NAMESPACE));
    strcpy(res->name, name);
    res->module = module;
    res->isModuleType = isModuleType;
    res->commands = new IvanLinkedList::LinkedList<DEM_COMMAND>();

    _namespaces.add(res);
  }

  return res;
}

DEM_NAMESPACE* DroneExecutionManager::createNamespace(const __FlashStringHelper* name, uint8_t module, boolean isModuleType) {
  char buffer[20];
  strcpy_P(buffer, (PGM_P)(name));
  return createNamespace(buffer, module, isModuleType);
}

void DroneExecutionManager::registerCommand(DEM_NAMESPACE* ns, const char* command, uint8_t dataType, DEMCommandHandler handler) {
  if (ns == NULL || command == NULL || handler == NULL) return;

  DEM_COMMAND temp;
  temp.str = command;
  temp.dataType = dataType;
  temp.handler = handler;
  ns->commands->add(temp);
}

DEM_COMMAND DroneExecutionManager::getCommand(DEM_NAMESPACE* ns, const char* command) {
  DEM_COMMAND cmd;
  Log.noticeln(F("[DEM.gC] Find handler for %s in %s\n"), command, ns->name);
  if (ns != NULL) {
    for(int i = 0; i < ns->commands->size(); i++){
      cmd = ns->commands->get(i);
      if (strcmp(cmd.str, command) == 0) {
        Log.noticeln(F("[DEM.gC] Found matching handler for %s\n"), command);
        return cmd;
      }
    }
  }
  Log.errorln(F("[DEM.gC] No handler found"));
  cmd.handler = NULL;
  cmd.str = NULL;
  return cmd;
}


void DroneExecutionManager::callStackPush(DEM_CALLSTACK_ENTRY entry) {
  if (_call.p < DEM_CALLSTACK_SIZE-1) {
    _call.p++;
    _call.stack[_call.p] = entry;
  } else {
    Log.errorln(F("[DEM.cSP] Call stack full"));
  }
}


void DroneExecutionManager::callStackPop() {
  if (_call.p > -1) _call.p--;
}


void DroneExecutionManager::dataStackPush(uint8_t d) {
  if (_data.p < DEM_DATASTACK_SIZE-1) {
    _data.p++;
    _data.stack[_data.p] = d;
  } else {
    Log.errorln(F("[DEM.dSP] Data stack full"));
  }
}

uint8_t DroneExecutionManager::dataStackPop() {
  if (_data.p > -1) {
    return _data.stack[_data.p];
    _data.p--;
  }
  return 0;
}

// peek at an item offset down from the top of the stack
uint8_t DroneExecutionManager::dataStackPeek(uint8_t offset) {
  int p = _data.p - offset;
  if ( p >-1) {
    return _data.stack[p];
  } else {
    return 0;
  }
}


void DroneExecutionManager::printInstruction(DEM_INSTRUCTION * instruction) {
  //Serial.print("NS: "); Serial.print(instruction->ns);
  Serial.print(", C: "); Serial.print(instruction->command[0]); Serial.print(instruction->command[1]);
  Serial.print(", ");
  Serial.print(instruction->addr.node);
  Serial.print('>');
  Serial.print(instruction->addr.channel);
  Serial.print('.');
  Serial.print(instruction->addr.param);
  Serial.print(' ');
  uint8_t ty = instruction->dataType;
  switch(ty) {
    case DRONE_LINK_MSG_TYPE_UINT8_T: Serial.print(F("uint8_t")); break;
    case DRONE_LINK_MSG_TYPE_ADDR: Serial.print(F("addr")); break;
    case DRONE_LINK_MSG_TYPE_UINT32_T: Serial.print(F("uint32_t")); break;
    case DRONE_LINK_MSG_TYPE_FLOAT: Serial.print(F("Float")); break;
    case DRONE_LINK_MSG_TYPE_CHAR: Serial.print(F("Char")); break;
    case DRONE_LINK_MSG_TYPE_NAME: Serial.print(F("Name")); break;
    case DRONE_LINK_MSG_TYPE_NAMEQUERY: Serial.print(F("NameQuery")); break;
    case DRONE_LINK_MSG_TYPE_QUERY: Serial.print(F("Query")); break;
  }
  Serial.print(' ');
  uint8_t len = instruction->numTokens;
  Serial.print("numTokens: "); Serial.println(len);
  for (uint8_t i=0; i<instruction->numTokens; i++) {
    Serial.print(i);
    Serial.print(": ");
    if (_instruction.tokens[i].isEnum) {
      Serial.print(_instruction.tokens[i].txt);
    } else {
      switch(ty) {
        case DRONE_LINK_MSG_TYPE_UINT8_T: Serial.print(_instruction.tokens[i].value.uint8); break;
        case DRONE_LINK_MSG_TYPE_ADDR: //TODO break;
        case DRONE_LINK_MSG_TYPE_UINT32_T: Serial.print(_instruction.tokens[i].value.uint32); break;
        case DRONE_LINK_MSG_TYPE_FLOAT: Serial.print(_instruction.tokens[i].value.f); break;
        case DRONE_LINK_MSG_TYPE_CHAR: Serial.print(_instruction.tokens[i].value.c); break;
      }
    }
    Serial.print('\n');
  }
  Serial.print("\n\n");
}


void DroneExecutionManager::parseEnums(DEM_INSTRUCTION * instruction, DEM_ENUM_MAPPING * mappingTable, uint16_t mappings) {
  DEM_TOKEN * t;
  boolean parsed;
  for (uint8_t i=0; i<instruction->numTokens; i++) {
    t = &instruction->tokens[i];
    parsed = false;

    if (t->isEnum) {
      // attempt to parse and convert to literal value
      for (uint8_t j=0; j<mappings; j++) {
        if (strcmp_P(t->txt, mappingTable[j].str) == 0) {
          t->value.uint8 = mappingTable[j].value;
          parsed = true;
        }
      }
    }

    if (parsed) {
      t->isEnum = false;
    }
  }
}


boolean DroneExecutionManager::load(const char * filename) {
  if (_file) {
    _file.close();
  }

  Log.noticeln(F("[DEM.l] %s ..."), filename);
  if (SPIFFS.exists(filename)) {
    uint32_t startTime = millis();

    _file = SPIFFS.open(filename, FILE_READ);

    if(!_file){
        Log.errorln(F("[DEM.l] Error opening file"));
        return false;
    }

    //Log.noticeln(F("[DEM.load] File open, size: %u"), _file.size());
    // reset default namespace
    _instruction.ns = getNamespace("core");

    // crete new macro
    DEM_MACRO *m = createMacro(filename);

    // TODO - parse, compile and store
    char readBuffer[128];
    uint8_t rp =0;  //read buffer position
    DEM_INSTRUCTION_COMPILED instr;

    while(_file.available()) {
      char c = _file.read();
      switch(c) {
        case '\n':
          // end of line, parse the readBuffer
          if (rp > 0) {
            // ensure line is terminated
            readBuffer[rp] = 0;
            //Serial.println("");
            //Serial.println("[DEM] Compiling line...");
            if (compileLine(readBuffer, &instr)) {
              // store the compiled line
              m->commands->add(instr);
            }
          }
          rp = 0;
          break;
        case '\r': break; // ignore
        default:
          if (rp < 126) {
            readBuffer[rp] = c;
            Serial.print(c);
            rp++;
          } else {
            Log.errorln("[DEM.l] readBuffer full");
          }

      }
    }

    //

    uint32_t duration = millis() - startTime;
    Log.noticeln(F("[DEM.l] Complete: %u commands, %u ms"), m->commands->size(), duration);

    return true;
  } else {
    Log.errorln(F("[DEM.l] %s file does not exist"), filename);
    return false;
  }
}


boolean DroneExecutionManager::tokenContainsNumber(char tokenStart) {
  return tokenStart == 45 || (tokenStart >= 48 && tokenStart <= 57);
}


boolean DroneExecutionManager::compileLine(const char * line, DEM_INSTRUCTION_COMPILED* instr) {
  //uint32_t startTime = millis();

  if (line[0] != 0) {
    char token[DEM_TOKEN_LENGTH+1];
    uint8_t tokenLen = 0;
    _instruction.numTokens = 0;
    //uint8_t numTokens = 0;
    boolean eot = false; // end of token
    boolean eos = false; // end of string
    boolean inStr = false;  // parsing a string
    boolean att = false;  // add to token
    boolean eon = false;  // end of namespace
    boolean inType = false;  // in type name
    boolean gotType = false;
    boolean eoty = false; // end of type
    boolean inComment = false;
    boolean inAddr = false;
    boolean error = false;  // error parsing line - treat as invalid and skip
    boolean valid = false;

    boolean eono = false; // end of node
    boolean eoc = false;  // end of channel
    boolean eop = false; //end of param

    char c;
    uint8_t cp = 0; // character position
    boolean eol = false;
    do {
      if (line[cp] > 0) { c = line[cp]; }
      else { c = '\n'; } // dummy char to finish a file cleanly

      //Serial.print(c);

      att = false;
      if (!error && !inComment) {
        switch(c) {
          // quotes
          case '"': if (inStr) { eos = true; eot = true; inStr=false; } else { inStr = true; eot = true; } break;

          // namespace (if received before a valid command)
          // end of channel if inAddr
          // add to str if inStr or valid cmd (e.g. numeric)
          case '.': if (inStr ) { att = true; }
          else if (inAddr) {
            eoc = true;
            eot = true;
          }
          else if (valid) {
            if (tokenLen == 0) {
              // short-hand parameter address
              inAddr = true;
            } else {
              // probably part of a numeric value
              att =true;
            }
          }
          else { eon = true; eot = true; } break;

          // detect address
          case '>': if (inStr) { att = true; }
          else { inAddr = true; eot=true; eono = true; } break;

          // comments
          case '/':
          case '#':
            if (inStr) { att = true; } else { eot=true; inComment = true; } break;

          // type name
          case '(': if (inStr) { att = true; } else { eot = true; inType = true; } break;
          case ')': if (inStr) { att = true; } else {
            if (inType) {
              inType = false;
              eoty = true;
            } else {
              error = true;
            }
            eot = true;
          } break;

          //whitespace
          case ' ':
          case '\t':
            if (inStr) { att = true; }
            else {
              if (inAddr) eop = true;
              eot=true;
            }; break;

          case '\r': eot = true; break;

          // end of line
          case '\n':
            eot = true; eol = true;
            break;

          default:
            if (c >= 32 && c <=126) att = true;
        }
      } else {
        // catch eol
        if (c == '\n') { eol = true; }
      }

      if (att) {
        token[tokenLen] = c;
        tokenLen++;

        // check for buffer overruns
        if (tokenLen == 16) eot = true;
      }

      // token has been extract - now to parse it
      if (eot) {
        if (tokenLen > 0) {
          // null terminate
          token[tokenLen] = 0;
          // decode token
          // address component
          if (inAddr) {
            if (eono) {
              eono = false;
              if (token[0] == '@') {
                _instruction.addr.node = _dlm->node();
              } else if (tokenContainsNumber(token[0])) {
                _instruction.addr.node = atoi(token);
              } else {
                // named node
                if (strcmp(token, _dmm->hostname().c_str()) == 0) {
                  _instruction.addr.node = _dlm->node();
                } else
                _instruction.addr.node = _dlm->getNodeByName(token);
                Serial.print("_Node lookup_");
              }
              Serial.print("_N_");
            }
            else if (eoc) {
              eoc = false;
              if (token[0] == '@') {
                _instruction.addr.channel = _channelContext;
              } else if (tokenContainsNumber(token[0])) {
                _instruction.addr.channel = atoi(token);
              } else {
                // named channel
                // TODO:
                _instruction.addr.channel = 0;
                Serial.print("_Channel lookup_");
              }
              Serial.print("_C_");
            }
            else if (eop) {
              eop = false;
              inAddr = false;
              if (tokenContainsNumber(token[0])) {
                _instruction.addr.param = atoi(token);
              } else {
                // named param
                // TODO:
                _instruction.addr.param = 0;
                Serial.print("_Param lookup_");
              }
              Serial.print("_P_");
            } else {
              Log.errorln("Error decoding address");
              error = 1;
            }
          }
          // namespace
          else if (eon) {
            Serial.print("_NS_");
            if (tokenLen == 0) {
              _instruction.ns = getNamespace("core");
            } else
              _instruction.ns = getNamespace(token);
            eon = false;
          }
          // string
          else if (eos) {
            Serial.print("_STR_");
            eos = false;
            if (_instruction.numTokens ==0) {
              for (uint8_t i=0; i<tokenLen; i++) {
                _instruction.tokens[i].value.c = token[i];
                _instruction.tokens[i].isEnum = false;
              }
              _instruction.numTokens = tokenLen;
              _instruction.dataType= DRONE_LINK_MSG_TYPE_CHAR;
              gotType = true;
            } else {
              Log.errorln("Values already written, but string found");
              error = true;
            }
          }
          // type
          else if (eoty) {
            Serial.print("_TYPE_");
            if (token[0] == 'f') {
              Serial.print("_f_");
              _instruction.dataType = DRONE_LINK_MSG_TYPE_FLOAT;
            } else if (token[0] == 'a') {
              // TODO - how is this meant to work???
              Serial.print("_a_");
              _instruction.dataType = DRONE_LINK_MSG_TYPE_ADDR;
            } else if (token[0] == 'c') {
              Serial.print("_c_");
              _instruction.dataType = DRONE_LINK_MSG_TYPE_CHAR;
            } else if (token[0] == 'u') {
              if (token[1] == '8') {
                 Serial.print("_u8_");
                 _instruction.dataType = DRONE_LINK_MSG_TYPE_UINT8_T;
              } else {
                // assume u32
                Serial.print("_u32_");
                _instruction.dataType = DRONE_LINK_MSG_TYPE_UINT32_T;
              }
            }
            gotType = true;
            eoty = false;
          }
          // generic token
          else {
            // if first generic token, then it's a command
            if (!valid) {
              if (!inAddr) {
                valid = true;
                strcpy(_instruction.command, token);

                if (_instruction.ns == 0) {
                  _instruction.ns = getNamespace("core");
                }

                DEM_COMMAND temp = getCommand(_instruction.ns, _instruction.command);
                instr->handler = temp.handler;
                instr->ns = _instruction.ns;
                if (gotType) {
                  // TODO:??
                  if (temp.dataType<255) {
                    if (_instruction.dataType != temp.dataType) {
                      Log.errorln("[DEM.cL] type mismatch %u > %u", _instruction.dataType,temp.dataType);
                    }
                  }
                } else {
                  if (temp.dataType == 255) {
                    Log.errorln("[DEM.cL] unknown data type");
                  } else
                    _instruction.dataType = temp.dataType;
                }
                instr->cmd = temp.str;
              } else {
                // invalid command
                Log.errorln("Invalid command");
                error = true;
              }
              Serial.print("_CMD_");
            } else {

              // attempt to decode token

              // addresses
              if (inAddr) {
                // this should already have populated the msg address
                inAddr = false;
                Serial.print("_ADDR_");
              }
              // numbers
              else if (tokenContainsNumber(token[0])) {
                // appears to be a numeric value
                float fv = atof(token);
                Serial.print("_"); Serial.print(fv); Serial.print("_");

                if (_instruction.numTokens < DRONE_LINK_MSG_MAX_PAYLOAD) {
                  switch (_instruction.dataType) {
                    case DRONE_LINK_MSG_TYPE_UINT8_T:
                      _instruction.tokens[_instruction.numTokens].value.uint8 = fv;
                      _instruction.tokens[_instruction.numTokens].isEnum = false;
                      break;
                    case DRONE_LINK_MSG_TYPE_UINT32_T:
                      _instruction.tokens[_instruction.numTokens].value.uint32 = fv;
                      _instruction.tokens[_instruction.numTokens].isEnum = false;
                      break;
                    case DRONE_LINK_MSG_TYPE_FLOAT:
                      _instruction.tokens[_instruction.numTokens].value.f = fv;
                      _instruction.tokens[_instruction.numTokens].isEnum = false;
                      break;
                    case DRONE_LINK_MSG_TYPE_ADDR:
                      // TODO
                      break;
                    case DRONE_LINK_MSG_TYPE_CHAR:
                      // should never happen
                      break;
                  }
                  _instruction.numTokens++;
                } else {
                  Log.errorln("Too many values!");
                  error = true;
                }


              }
              // names a param, enum, etc
              else {
                Serial.print("_ENUM_");
                if (tokenLen <= DEM_TOKEN_LENGTH) {
                  strcpy(_instruction.tokens[_instruction.numTokens].txt, token);
                  _instruction.tokens[_instruction.numTokens].isEnum = true;
                  _instruction.numTokens++;
                } else {
                  Log.errorln("Enum string too long");
                }
              }
            }
          }

          Serial.print("[");
          Serial.print(token);
          Serial.println("]");

          tokenLen = 0;

        } else {
          // catch zero length namespace
          if (eon) {
            Serial.print("_NS_");
            _instruction.ns = getNamespace("core");
            eon = false;
          }
        }

        eot = false;
      }

      cp++;
    } while (!eol);

    Serial.println("");

    if (valid && !error) {

      // parse global enums
      parseEnums(&_instruction, (DEM_ENUM_MAPPING*)&DEM_ENUM_TABLE, sizeof(DEM_ENUM_TABLE)/sizeof(DEM_ENUM_MAPPING));

      // parse DroneModule params
      parseEnums(&_instruction, (DEM_ENUM_MAPPING*)&DRONE_PARAM_TABLE, sizeof(DRONE_PARAM_TABLE)/sizeof(DEM_ENUM_MAPPING));

      // parse namespace enums
      // TODO

      // parse command specific enums
      // TODO

      //Log.noticeln("Decoded instruction: (%u)", _instruction.numTokens);
      //printInstruction(&_instruction);

      // compile the rest of the instruction
      if (instr->handler != NULL) {

        instr->msg.node = _instruction.addr.node;
        instr->msg.channel = _instruction.addr.channel;
        instr->msg.param = _instruction.addr.param;
        uint8_t byteLen = DRONE_LINK_MSG_TYPE_SIZES[_instruction.dataType] * _instruction.numTokens;
        instr->msg.payload.c[0] = 0; // safety
        if (byteLen <= DRONE_LINK_MSG_MAX_PAYLOAD) {
          instr->msg.paramTypeLength = DRONE_LINK_MSG_WRITABLE | ((_instruction.dataType & 0x7) << 4) | (byteLen-1);
          for(uint8_t i=0; i<_instruction.numTokens; i++) {
            switch(_instruction.dataType) {
              case DRONE_LINK_MSG_TYPE_UINT8_T:
                instr->msg.payload.uint8[i] = _instruction.tokens[i].value.uint8;
                break;
              case DRONE_LINK_MSG_TYPE_ADDR:
                instr->msg.payload.addr[i] = _instruction.tokens[i].value.addr;
                break;
              case DRONE_LINK_MSG_TYPE_UINT32_T:
                instr->msg.payload.uint32[i] = _instruction.tokens[i].value.uint32;
                break;
              case DRONE_LINK_MSG_TYPE_FLOAT:
                instr->msg.payload.f[i] = _instruction.tokens[i].value.f;
                break;
              case DRONE_LINK_MSG_TYPE_CHAR:
                instr->msg.payload.c[i] = _instruction.tokens[i].value.c;
                break;
            }
          }
        } else {
          valid = false;
          error = true;
          Log.errorln(F("[DEM.cL] Compiled message exceeds 16 bytes"));
        }
      } else {
        valid = false;
        error = true;
        Log.errorln(F("[DEM.cL] Unknown command"));
      }

    }

    //int32_t elapsed = millis() - startTime;
    return (valid && !error);
    //Log.noticeln(F("compiled in: %u ms"), elapsed);
  }

  return false;
}


void DroneExecutionManager::execute() {

  if (_safeMode) return;

  // if we have an active macro
  if (_call.p > -1) {
    DEM_CALLSTACK_ENTRY *cse = &_call.stack[_call.p];
    // check valid macro pointer
    if (cse->macro) {
      // check we haven't reached the end of the macro
      if (cse->i < cse->macro->commands->size()) {
        // fetch the instruction
        DEM_INSTRUCTION_COMPILED ic = cse->macro->commands->get(cse->i);

        Log.noticeln(F("[DEM.e] %s #%u > %s\n"), cse->macro->name, cse->i, ic.cmd);

        // TODO: print when continuing

        // execute the instruction
        if (ic.handler(&ic, &_call, &_data, cse->continuation)) {
          // command completed
          // increment instruction pointer
          cse->i++;
        } else {
          // command is still busy executing
          cse->continuation = true;
        }
      } else {
        // pop the call stack
        callStackPop();
      }
    }
  }
}


void DroneExecutionManager::serveMacroInfo(AsyncWebServerRequest *request) {

  AsyncResponseStream *response = request->beginResponseStream("text/text");
  response->addHeader("Server","ESP Async Web Server");
  response->print(F("Macros: \n"));

  DEM_MACRO* m;
  for(int i = 0; i < _macros.size(); i++){
    m = _macros.get(i);
    response->printf("%u: %s - %u commands\n", i, m->name, m->commands->size());

    // now print all associated commands
    DEM_INSTRUCTION_COMPILED ic;
    for (uint8_t j=0; j<m->commands->size(); j++) {
      ic = m->commands->get(j);
      response->printf("   %u: %s\n", j, ic.cmd);
    }
    response->print("\n");
  }

  //send the response last
  request->send(response);
}


void DroneExecutionManager::serveCommandInfo(AsyncWebServerRequest *request) {

  AsyncResponseStream *response = request->beginResponseStream("text/text");
  response->addHeader("Server","ESP Async Web Server");
  response->print(F("Namespaces and Commands: \n"));

  DEM_NAMESPACE* ns;
  for(uint8_t i = 0; i < _namespaces.size(); i++){
    ns = _namespaces.get(i);
    response->printf("%u: %s - %u commands\n", i, ns->name, ns->commands->size());

    // now print all registred commands
    DEM_COMMAND c;
    for (uint8_t j=0; j<ns->commands->size(); j++) {
      c = ns->commands->get(j);
      response->printf("   %u: %s - %u\n", j, c.str, (c.handler != NULL ? 1 : 0));
    }
    response->print("\n");
  }

  //send the response last
  request->send(response);
}

void DroneExecutionManager::serveExecutionInfo(AsyncWebServerRequest *request) {

  if(request->hasParam("execute")) {
    // trigger immediate execution
    _safeMode = false;
  }

  AsyncResponseStream *response = request->beginResponseStream("text/text");
  response->addHeader("Server","ESP Async Web Server");
  response->print(F("Execution state: \n"));

  response->print(F("safeMode = "));
  if (_safeMode) {
    response->print(F("Yes\n"));
  } else {
    response->print(F("No\n"));
  }

  //send the response last
  request->send(response);
}


boolean DroneExecutionManager::core_done(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  Log.noticeln(F("[DEM.d]"));
  dataStackPop();
  return true;
}


boolean DroneExecutionManager::core_load(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  char buffer[DRONE_LINK_MSG_MAX_PAYLOAD+1];
  uint8_t len = (instr->msg.paramTypeLength & 0xF)+1;
  memcpy(buffer, instr->msg.payload.c, len);
  buffer[len] = 0;

  if (len > 1) {
    load(buffer);
  }
  return true;
}

boolean DroneExecutionManager::core_node(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  uint8_t id = instr->msg.payload.uint8[0];
  Log.noticeln(F("[.n] %u"), id);
  if (id > 0 && id < 255) {
    _dmm->node(id);
    _dlm->node(id);
  } else
    Log.errorln(F("[.n] invalid address"));
  return true;
}

boolean DroneExecutionManager::core_publish(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  uint8_t module = dataStackPeek(0);

  char buffer[DRONE_LINK_MSG_MAX_PAYLOAD+1];
  uint8_t len = (instr->msg.paramTypeLength & 0xF)+1;
  memcpy(buffer, instr->msg.payload.c, len);
  buffer[len] = 0;

  Log.noticeln(F("[.pub] Publishing %s on module %u"), buffer, module);
  // get param address by name from module
  DroneModule* mod = _dmm->getModuleById(module);
  if (mod != NULL) {
    DRONE_PARAM_ENTRY* p = mod->getParamEntryByName(buffer);
    if (p != NULL) {
      p->publish = true;

      } else {
        Log.errorln(F("[.r] Unknown param"));
      }

  } else {
      Log.errorln(F("[.pub] Unknown module"));
  }
  return true;
}

boolean DroneExecutionManager::core_restart(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  Log.noticeln(F("[.restart] Restarting node"));
  _dmm->restart();
  return true;
}

boolean DroneExecutionManager::core_run(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  char buffer[DRONE_LINK_MSG_MAX_PAYLOAD+1];
  uint8_t len = (instr->msg.paramTypeLength & 0xF)+1;
  memcpy(buffer, instr->msg.payload.c, len);
  buffer[len] = 0;

  if (len > 1) {
    DEM_CALLSTACK_ENTRY cse;
    cse.i=0;
    cse.macro = getMacro(buffer);
    cse.continuation = false;
    if (cse.macro != NULL) {
      Log.noticeln(F("[.r] Executing macro %s\n"), buffer);
      if (cse.macro->commands->size() > 0) {
        callStackPush(cse);
      } else {
        Log.errorln(F("[.r] Macro has no compiled commands"));
      }

    } else {
      Log.errorln(F("[.r] Unknown macro %s"), instr->msg.payload.c);
    }
  }
  return true;
}

boolean DroneExecutionManager::core_send(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  Log.noticeln(F("[.send] Publish msg to: %u>%u.%u\n"), instr->msg.node, instr->msg.channel, instr->msg.param);
  // check for address rewrites
  if (instr->msg.node == 0) instr->msg.node = _dlm->node();
  // see if we can get module context
  if (instr->msg.channel == 0) instr->msg.channel = dataStackPeek(0);
  DroneLinkMsg temp(&instr->msg);
  if (instr->msg.channel > 0 && instr->msg.param > 0) {
    instr->msg.source = instr->msg.node;
    _dlm->publish(temp);
  } else {
    Log.errorln(F("[.send] Invalid address"));
  }

  return true;
}

boolean DroneExecutionManager::core_setup(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  Log.noticeln(F("[.s] Completing module setup"));
  _dmm->setupModules();

  // if we made it to here, can assume no crashes on startup
  EEPROM.write(DEM_EEPROM_SUCCESSFUL_BOOT, DEM_BOOT_SUCCESS);
  EEPROM.commit();

  Log.noticeln(F("[.s] Setup complete"));
  return true;
}


/*
   Module Constructor
*/

boolean DroneExecutionManager::mod_constructor(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  uint8_t id = instr->msg.payload.uint8[0];
  Log.noticeln(F("[.c] Instantiating %s as %u"), instr->ns->name, id);

  if (id > 0 && id < 255) {
    DroneModule *newMod;

    if (strcmp_P(instr->ns->name, MANAGEMENT_STR_MANAGEMENT) == 0) {
      newMod = new ManagementModule(id, _dmm, _dlm, this);
    } if (strcmp_P(instr->ns->name, MOTOR_STR_MOTOR) == 0) {
      newMod = new MotorModule(id, _dmm, _dlm, this);
    } else {
      Log.errorln(F("[.c] Unknown type"));
    }

    if ( newMod ) {
      // push id onto data stack for use by param and publish commands
      dataStackPush(id);
    }
    /*
    if (typeName.equals(MANAGEMENT_STR_MANAGEMENT)) {
                newMod = new ManagementModule(id, this, _dlm);
              } else if (typeName.equals(TELEMETRY_STR_TELEMETRY)) {
                newMod = new TelemetryModule(id, this, _dlm);
              } else if (typeName.equals(PSTR("UDPTelemetry"))) {
                newMod = new UDPTelemetryModule(id, this, _dlm);
              } else if (typeName.equals(SERVO_STR_SERVO)) {
                newMod = new ServoModule(id, this, _dlm);
              } else if (typeName.equals(PSTR("Timer"))) {
                newMod = new TimerModule(id, this, _dlm);
              } else if (typeName.equals(PSTR("WindSpeedDir"))) {
                //newMod = new WindSpeedDirModule(id, this, _dlm);
              } else if (typeName.equals(BME280_STR_BME280)) {
                newMod = new BME280Module(id, this, _dlm);
              } else if (typeName.equals(INA219_STR_INA219)) {
                newMod = new INA219Module(id, this, _dlm);
              } else if (typeName.equals(HMC5883L_STR_HMC5883L)) {
                newMod = new HMC5883LModule(id, this, _dlm);
              } else if (typeName.equals(NMEA_STR_NMEA)) {
                newMod = new NMEAModule(id, this, _dlm);
              } else if (typeName.equals(MPU6050_STR_MPU6050)) {
                newMod = new MPU6050Module(id, this, _dlm);
              } else if (typeName.equals(MOTOR_STR_MOTOR)) {
                newMod = new MotorModule(id, this, _dlm);
              } else if (typeName.equals(TANK_STEER_STR_TANK_STEER)) {
                newMod = new TankSteerModule(id, this, _dlm);
              } else if (typeName.equals(BASIC_NAV_STR_BASIC_NAV)) {
                newMod = new BasicNavModule(id, this, _dlm);
              } else if (typeName.equals(WAYPOINT_NAV_STR_WAYPOINT_NAV)) {
                newMod = new WaypointNavModule(id, this, _dlm);
              } else if (typeName.equals(TURN_RATE_STR_TURN_RATE)) {
                newMod = new TurnRateModule(id, this, _dlm);
              } else if (typeName.equals(RFM69_TELEMETRY_STR_RFM69_TELEMETRY)) {
                newMod = new RFM69TelemetryModule(id, this, _dlm);
              } else if (typeName.equals(JOYSTICK_STR_JOYSTICK)) {
                newMod = new JoystickModule(id, this, _dlm);
              } else if (typeName.equals(OLED_STR_OLED)) {
                newMod = new OLEDModule(id, this, _dlm);
              } else if (typeName.equals(CONTROLLER_STR_CONTROLLER)) {
                newMod = new ControllerModule(id, this, _dlm);
              } else if (typeName.equals(NunJOYSTICK_STR_NunJOYSTICK)) {
                newMod = new NunchuckJoystick(id, this, _dlm);
              } else if (typeName.equals(NEOPIXEL_STR_NEOPIXEL)) {
                newMod = new NeopixelModule(id, this, _dlm);
              } else {
                Log.errorln(F("Unknown type"));
              }
    */
    Log.noticeln(F("[.c] done"));
  } else {
    Log.errorln(F("[.c] Invalid id"));
  }
  return true;
}


boolean DroneExecutionManager::mod_param(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  uint8_t module = dataStackPeek(0);
  Log.noticeln(F("[.p] Setting param %s on module %u"), instr->cmd, module);
  // get param address by name from module
  DroneModule* mod = _dmm->getModuleById(module);
  if (mod != NULL) {
    uint8_t param = mod->getParamIdByName(instr->cmd);

    if (param != 0) {
      // set param
      if (instr->msg.node == 0) instr->msg.node = _dlm->node();
      instr->msg.source = instr->msg.node;
      instr->msg.channel = module;
      instr->msg.param = param;

      DroneLinkMsg temp(&instr->msg);
      temp.print();
      _dlm->publish(temp);
    } else {
      Log.errorln(F("[.p] Unknown param"));
    }

  } else {
    Log.errorln(F("[.p] Unable to locate module"));
  }
  return true;
}
