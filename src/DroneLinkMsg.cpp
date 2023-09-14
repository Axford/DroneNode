#include "DroneLinkMsg.h"



uint8_t setDroneLinkMsgPriorityParam(uint8_t pri, uint8_t param) {
  return pri << 6 | (param & 0b00111111);
}

uint8_t getDroneLinkMsgParam(uint8_t paramPriority) {
  return paramPriority & 0b00111111;
}

uint8_t getDroneLinkMsgPriority(uint8_t paramPriority) {
  return paramPriority >> 6;
}




DroneLinkMsg::DroneLinkMsg(DroneLinkMsg *msg) {
  _msg = msg->_msg;
}

DroneLinkMsg::DroneLinkMsg(DRONE_LINK_MSG *msg) {
  memcpy(&_msg, msg, sizeof(_msg));
}

DroneLinkMsg::~DroneLinkMsg() { }

 bool DroneLinkMsg::matchParam(uint8_t paramMask) {
  //Serial.print("Mask:");
  //Serial.println(paramMask);
  return (param() == paramMask) || (paramMask == DRONE_LINK_PARAM_ALL);
}

// util
uint8_t DroneLinkMsg::packParamLength(boolean w, uint8_t t, uint8_t l) {
  return (w ? DRONE_LINK_MSG_WRITABLE : 0) | ((t & 0x7) << 4) | ((l-1) & 0x0F);
}

// setters
void DroneLinkMsg::source(uint8_t v) { _msg.source =v; }
void DroneLinkMsg::node(uint8_t v) { _msg.node = v; }
void DroneLinkMsg::channel(uint8_t v) { _msg.channel = v; }
void DroneLinkMsg::param(uint8_t v) { _msg.paramPriority = (_msg.paramPriority & 0b11000000) | v; }
void DroneLinkMsg::priority(uint8_t v) { _msg.paramPriority = (_msg.paramPriority & 0b00111111) | (v << 6); }
void DroneLinkMsg::type(uint8_t v) {
  _msg.paramTypeLength = ((v & 0x7) << 4) | (_msg.paramTypeLength & 0b10001111);
}
void DroneLinkMsg::length(uint8_t v) {
  _msg.paramTypeLength = (v-1) | (_msg.paramTypeLength & 0xF0);
}
void DroneLinkMsg::writable(boolean v) {
  if (v) {
    _msg.paramTypeLength = _msg.paramTypeLength | DRONE_LINK_MSG_WRITABLE;
  } else {
    _msg.paramTypeLength = _msg.paramTypeLength & (~DRONE_LINK_MSG_WRITABLE);
  }
}
void DroneLinkMsg::setUint8_t(uint8_t v) {
  _msg.payload.uint8[0] = v;
}
void DroneLinkMsg::setFloat(float v) {
  _msg.payload.f[0] = v;
  //memcpy(_msg.payload, &v, 4);
  /*
  Serial.print("raw float: ");
  for (uint8_t i=0; i<4; i++) {
    Serial.print(_msg.payload[i], HEX);
    Serial.print(" ");
  }
  Serial.print("\n");
  */
}

// getters
uint8_t DroneLinkMsg::source() { return _msg.source; }
uint8_t DroneLinkMsg::node() { return _msg.node; }
uint8_t DroneLinkMsg::channel() { return _msg.channel; }
uint8_t DroneLinkMsg::param() { return _msg.paramPriority & 0b00111111; }
uint8_t DroneLinkMsg::priority() { return _msg.paramPriority >> 6; }
boolean DroneLinkMsg::writable() { return (_msg.paramTypeLength & DRONE_LINK_MSG_WRITABLE) > 0; }
uint8_t DroneLinkMsg::type() {
  return (_msg.paramTypeLength >> 4) & 0x07;
}
uint8_t DroneLinkMsg::length() {
  return (_msg.paramTypeLength & 0x0F) + 1;
}
uint8_t DroneLinkMsg::totalSize() {
  return length() + 5;
}

// utility

boolean DroneLinkMsg::sameSignature(DroneLinkMsg *msg) {
  // returns true if channel, param and type match
  return memcmp(&_msg, &msg->_msg, 5) == 0;
  /*
  return (source() == msg->source()) &&
         (node() == msg->node()) &&
         (channel() == msg->channel()) &&
         (param() == msg->param()) &&
         (type() == msg->type());
        */
}

void DroneLinkMsg::printAddress(DRONE_LINK_ADDR *addr) {
  Serial.print(addr->source);
  Serial.print(':');
  Serial.print(addr->node);
  Serial.print('>');
  Serial.print(addr->channel);
  Serial.print('.');
  Serial.print(addr->paramPriority & 0b00111111);
}

void DroneLinkMsg::printAddress(DRONE_LINK_ADDR *addr, AsyncResponseStream *response) {
  response->print(addr->source);
  response->print(':');
  response->print(addr->node);
  response->print('>');
  response->print(addr->channel);
  response->print('.');
  response->print(addr->paramPriority & 0b00111111);
}

void DroneLinkMsg::print() {
  Serial.print(source());
  Serial.print(':');
  Serial.print(node());
  Serial.print('>');
  Serial.print(channel());
  Serial.print('.');
  Serial.print(param());
  Serial.print(' ');
  printPayload(&_msg.payload, _msg.paramTypeLength);
}

void DroneLinkMsg::printPayload(DRONE_LINK_PAYLOAD *payload, uint8_t paramTypeLength) {
  uint8_t ty = (paramTypeLength >> 4) & 0x7;
  uint8_t len = (paramTypeLength & 0xF) + 1;
  switch(ty) {
    case DRONE_LINK_MSG_TYPE_UINT8_T: Serial.print(F("u8")); break;
    case DRONE_LINK_MSG_TYPE_ADDR: Serial.print(F("a")); break;
    case DRONE_LINK_MSG_TYPE_UINT32_T: Serial.print(F("u32")); break;
    case DRONE_LINK_MSG_TYPE_FLOAT: Serial.print(F("f")); break;
    case DRONE_LINK_MSG_TYPE_CHAR: Serial.print(F("c")); break;
    case DRONE_LINK_MSG_TYPE_NAME: Serial.print(F("N")); break;
    case DRONE_LINK_MSG_TYPE_NAMEQUERY: Serial.print(F("NQ")); break;
    case DRONE_LINK_MSG_TYPE_QUERY: Serial.print(F("Q")); break;
  }
  if (ty != DRONE_LINK_MSG_TYPE_QUERY) {
    Serial.print('[');
    Serial.print(len);
    if ((paramTypeLength & DRONE_LINK_MSG_WRITABLE)>0) Serial.print(",W");
    Serial.print(']');
    Serial.print('=');
  }

  uint8_t numValues = len / DRONE_LINK_MSG_TYPE_SIZES[ty];
  for (uint8_t i=0; i<numValues; i++) {
    switch(ty) {
      case DRONE_LINK_MSG_TYPE_UINT8_T: Serial.print(payload->uint8[i]); break;
      case DRONE_LINK_MSG_TYPE_UINT32_T: Serial.print(payload->uint32[i]); break;
      case DRONE_LINK_MSG_TYPE_FLOAT: Serial.print(payload->f[i]); break;
      case DRONE_LINK_MSG_TYPE_CHAR: Serial.print(char(payload->c[i])); break;
      case DRONE_LINK_MSG_TYPE_ADDR: printAddress(&payload->addr[i]); break;
      //case DRONE_LINK_MSG_TYPE_QUERY: response->print(F("Query")); break;
    }
    if (ty != DRONE_LINK_MSG_TYPE_CHAR) Serial.print(' ');
  }
  Serial.print('\n');
}

void DroneLinkMsg::printPayload(DRONE_LINK_PAYLOAD *payload, uint8_t paramTypeLength, AsyncResponseStream *response) {

  uint8_t ty = (paramTypeLength >> 4) & 0x7;
  uint8_t len = (paramTypeLength & 0xF) + 1;
  switch(ty) {
    case DRONE_LINK_MSG_TYPE_UINT8_T: response->print(F("u8")); break;
    case DRONE_LINK_MSG_TYPE_ADDR: response->print(F("a")); break;
    case DRONE_LINK_MSG_TYPE_UINT32_T: response->print(F("u32")); break;
    case DRONE_LINK_MSG_TYPE_FLOAT: response->print(F("f")); break;
    case DRONE_LINK_MSG_TYPE_CHAR: response->print(F("c")); break;
    case DRONE_LINK_MSG_TYPE_NAME: response->print(F("N")); break;
    case DRONE_LINK_MSG_TYPE_NAMEQUERY: response->print(F("NQ")); break;
    case DRONE_LINK_MSG_TYPE_QUERY: response->print(F("Q")); break;
  default:
    response->print(F("?"));
  }
  if (ty != DRONE_LINK_MSG_TYPE_QUERY) {
    response->print('[');
    response->print(len);
    if ((paramTypeLength & DRONE_LINK_MSG_WRITABLE)>0) response->print(",W");
    response->print(']');
    response->print('=');
  }

  uint8_t numValues = len / DRONE_LINK_MSG_TYPE_SIZES[ty];
  for (uint8_t i=0; i<numValues; i++) {
    switch(ty) {
      case DRONE_LINK_MSG_TYPE_UINT8_T: response->print(payload->uint8[i]); break;
      case DRONE_LINK_MSG_TYPE_UINT32_T: response->print(payload->uint32[i]); break;
      case DRONE_LINK_MSG_TYPE_FLOAT: response->print(payload->f[i], 8); break;
      case DRONE_LINK_MSG_TYPE_CHAR: response->print(char(payload->c[i])); break;
      case DRONE_LINK_MSG_TYPE_ADDR: printAddress(&payload->addr[i], response); break;
    }
    if (ty != DRONE_LINK_MSG_TYPE_CHAR) response->print(' ');
  }
  response->print('\n');
}

boolean DroneLinkMsg::sameAddress(DRONE_LINK_ADDR *addr) {
  // returns true if channel, param and type match
  return (node() == addr->node &&
         channel() == addr->channel &&
         param() == (addr->paramPriority & 0b00111111));
}

// static utils
DRONE_LINK_ADDR DroneLinkMsg::parseAddress(String addr) {
  // of the form 1>2.3
  DRONE_LINK_ADDR res;
  // defaults
  res.node = 255;
  res.channel = 255;
  res.paramPriority = 255;

  // parse
  int gti = addr.indexOf(">");
  int pi = addr.indexOf(".");
  if (gti > -1 && pi > -1) {
    res.node = (addr.substring(0,gti)).toInt();
    res.channel = (addr.substring(gti+1,pi)).toInt();
    res.paramPriority = (addr.substring(pi+1)).toInt();
  }

  return res;
}
