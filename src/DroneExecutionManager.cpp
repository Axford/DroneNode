#include "DroneExecutionManager.h"
#include "DroneLinkManager.h"
#include "DroneModuleManager.h"
#include "pinConfig.h"
#include "strings.h"

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


DroneExecutionManager::DroneExecutionManager(DroneModuleManager *dmm, DroneLinkManager *dlm) {
  _dmm = dmm;
  _dlm = dlm;
  _call.p = -1;
  _data.p = -1;

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
  DEM_NAMESPACE *core = createNamespace(PSTR("core"),0);

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  registerCommand(core, PSTR("AL"), std::bind(&DroneExecutionManager::core_AL, this, _1, _2, _3, _4));
  registerCommand(core, PSTR("AM"), std::bind(&DroneExecutionManager::core_AM, this, _1, _2, _3, _4));

  registerCommand(core, PSTR("FC"), std::bind(&DroneExecutionManager::core_FC, this, _1, _2, _3, _4));
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

DEM_NAMESPACE* DroneExecutionManager::createNamespace(const char* name, uint8_t module) {
  DEM_NAMESPACE* res = getNamespace(name);

  if (res == NULL) {
    res =  (DEM_NAMESPACE*)malloc(sizeof(DEM_NAMESPACE));
    strcpy(res->name, name);
    res->module = module;
    res->commands = new IvanLinkedList::LinkedList<DEM_COMMAND>();

    _namespaces.add(res);
  }

  return res;
}

void DroneExecutionManager::registerCommand(DEM_NAMESPACE* ns, const char* command, DEMCommandHandler handler) {
  if (ns == NULL || command == NULL || handler == NULL) return;

  DEM_COMMAND temp;
  temp.str = command;
  temp.handler = handler;
  ns->commands->add(temp);
}

DEM_COMMAND DroneExecutionManager::getCommand(DEM_NAMESPACE* ns, const char* command) {
  DEM_COMMAND cmd;
  Serial.printf("Finding handler for %s\n", command);
  if (ns != NULL) {
    for(int i = 0; i < ns->commands->size(); i++){
      cmd = ns->commands->get(i);
      if (strcmp(cmd.str, command) == 0) {
        Serial.printf("Found matching handler for %s\n", command);
        return cmd;
      }
    }
  }
  Serial.println("No handler found");
  cmd.handler = NULL;
  cmd.str = NULL;
  return cmd;
}


void DroneExecutionManager::callStackPush(DEM_CALLSTACK_ENTRY entry) {
  if (_call.p < DEM_CALLSTACK_SIZE-1) {
    _call.p++;
    _call.stack[_call.p] = entry;
  } else {
    Log.errorln("Call stack full");
  }
}


void DroneExecutionManager::callStackPop() {
  if (_call.p > -1) _call.p--;
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

  Log.noticeln(F("[DEM] Load %s"), filename);
  if (SPIFFS.exists(filename)) {
    uint32_t startTime = millis();

    _file = SPIFFS.open(filename, FILE_READ);

    if(!_file){
        Log.errorln(F("[DEM] Error opening file"));
        return false;
    }

    Log.noticeln(F("[DEM] File open, size: %u"), _file.size());

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
            Serial.println("");
            Serial.println("Compiling line...");
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
            Log.errorln("buffer limit reached in load");
          }

      }
    }

    //

    uint32_t duration = millis() - startTime;
    Log.noticeln(F("[DEM] Load complete: %u commands, %u ms"), m->commands->size(), duration);

    return true;
  } else {
    Log.errorln(F("[DEM] %s file does not exist"), filename);
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
                // named node
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
            // TODO
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

            eoty = false;
          }
          // generic token
          else {
            // if first generic token, then it's a command
            if (!valid) {
              if (tokenLen == 2 && !inAddr) {
                valid = true;
              } else {
                // invalid command
                Log.errorln("Invalid command");
                error = true;
              }
              Serial.print("_CMD_");
              _instruction.command[0] = token[0];
              _instruction.command[1] = token[1];
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

      // parse namespace enums
      // TODO

      // parse command specific enums
      // TODO

      Log.noticeln("Decoded instruction: (%u)", _instruction.numTokens);
      printInstruction(&_instruction);

      // parse the command

      if (_instruction.ns == 0) {
        _instruction.ns = getNamespace("core");
      }
      DEM_COMMAND temp = getCommand(_instruction.ns, _instruction.command);
      instr->handler = temp.handler;
      instr->cmd = temp.str;
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
          Log.errorln("Compiled message exceeds 16 bytes");
        }
      } else {
        valid = false;
        error = true;
        Log.errorln("Unknown command");
      }

    }

    //int32_t elapsed = millis() - startTime;
    return (valid && !error);
    //Log.noticeln(F("compiled in: %u ms"), elapsed);
  }

  return false;
}


void DroneExecutionManager::execute() {
  // if we have an active macro
  if (_call.p > -1) {
    DEM_CALLSTACK_ENTRY *cse = &_call.stack[_call.p];
    // check valid macro pointer
    if (cse->macro) {
      // check we haven't reached the end of the macro
      if (cse->i < cse->macro->commands->size()) {
        Serial.printf("Executing instruction %u from macro %s\n", cse->i, cse->macro->name);
        // fetch the instruction
        DEM_INSTRUCTION_COMPILED ic = cse->macro->commands->get(cse->i);

        Serial.printf("instruction fetched %s (%u)\n", ic.cmd, (uint32_t)&ic.handler);

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


boolean DroneExecutionManager::core_AL(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  Serial.println("core_AL");
  char buffer[DRONE_LINK_MSG_MAX_PAYLOAD+1];
  uint8_t len = (instr->msg.paramTypeLength & 0xF)+1;
  memcpy(buffer, instr->msg.payload.c, len);
  buffer[len] = 0;

  if (len > 1) {
    load(buffer);
  }
  return true;
}

boolean DroneExecutionManager::core_AM(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  Serial.println("core_AM");
  Serial.printf("Publish msg to: %u>%u.%u", instr->msg.node, instr->msg.channel, instr->msg.param);
  DroneLinkMsg temp(&instr->msg);
  if (instr->msg.node > 0 && instr->msg.channel > 0 && instr->msg.param  > 0) {
    _dlm->publish(temp);
  } else {
    Serial.println("Invalid address");
  }

  return true;
}

boolean DroneExecutionManager::core_FC(DEM_INSTRUCTION_COMPILED* instr, DEM_CALLSTACK* cs, DEM_DATASTACK* ds, boolean continuation) {
  Serial.println("core_FC");
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
      Serial.printf("Executing macro %s\n", buffer);
      if (cse.macro->commands->size() > 0) {
        callStackPush(cse);
      } else {
        Serial.println("Macro has no compiled commands");
      }

    } else {
      Log.errorln("Unknown macro %s", instr->msg.payload.c);
    }
  }
  return true;
}
