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
  { "OUT0_0", PIN_OUT0_0},
  { "OUT0_1", PIN_OUT0_1 },
  { "OUT1_0", PIN_OUT1_0 },
  { "OUT1_1", PIN_OUT1_1 },
  { "OUT2_0", PIN_OUT2_0 },
  { "OUT2_1", PIN_OUT2_1 },
  { "DAC0_0", PIN_DAC0_0 },
  { "DAC0_1", PIN_DAC0_1 },
  { "IN0_0", PIN_IN0_0 },
  { "IN0_1", PIN_IN0_1 }
};


DroneExecutionManager::DroneExecutionManager(DroneModuleManager *dmm, DroneLinkManager *dlm) {
  _dmm = dmm;
  _dlm = dlm;
  _scriptLoaded = false;
  //_filePos = 0;

  _instruction.ns = 0;
  _instruction.command[0] = '_';
  _instruction.command[1] = '_';
  _instruction.addr.node = dlm->node();
  _instruction.addr.channel = 0;
  _instruction.addr.param = 0;
  _instruction.dataType = DRONE_LINK_MSG_TYPE_UINT8_T;
  _instruction.numTokens = 0;

  _channelContext = 0;
}


void DroneExecutionManager::printInstruction(DEM_INSTRUCTION * instruction) {
  Serial.print("NS: "); Serial.print(instruction->ns);
  Serial.print(", C: "); Serial.print(instruction->command[0]); Serial.print(instruction->command[1]);
  Serial.print(", ");
  Serial.print(instruction->addr.node);
  Serial.print('>');
  Serial.print(instruction->addr.channel);
  Serial.print('.');
  Serial.print(instruction->addr.param);
  Serial.print(' ');
  uint8_t ty = (instruction->dataType >> 4) & 0x7;
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
    _file = SPIFFS.open(filename, FILE_READ);

    if(!_file){
        Log.errorln(F("[DEM] Error opening file"));
        return false;
    }

    // script seems legit
    _filename = filename;
    _scriptLoaded = true;

    Log.noticeln(F("[DEM] File open, size: %u"), _file.size());

    return true;
  } else {
    Log.errorln(F("[DEM] %s file does not exist"), filename);
    return false;
  }
}


boolean DroneExecutionManager::tokenContainsNumber(char tokenStart) {
  return tokenStart == 45 || (tokenStart >= 48 && tokenStart <= 57);
}


void DroneExecutionManager::executeNextLine() {

  //if (!_scriptLoaded) return;

  if (_file) {
    Log.noticeln("[DEM] nextLine:");

    // continue reading next line
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
    boolean eol = false;
    do {
      if (_file.available()) { c = _file.read(); }
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
          else if (valid) { att =true; }
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
            _instruction.ns = 0;
            eon = false;
          }
        }

        eot = false;
      }

    } while (_file.available() && !eol);

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
    }

    if (_file.available() ==0) {
      Log.noticeln("[DEM] reached EOF");
      _scriptLoaded = false;
      _file.close();
    }
  }
}


void DroneExecutionManager::loop() {
  //Log.noticeln("log");
  executeNextLine();
}
