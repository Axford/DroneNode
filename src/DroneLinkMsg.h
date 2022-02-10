#ifndef DRONE_LINK_MSG_H
#define DRONE_LINK_MSG_H

#include "Arduino.h"
#include <ArduinoLog.h>
#include <ESPAsyncWebServer.h>

#define DRONE_LINK_SOURCE_OWNER       0  // indicates this packet originated with the param owner (i.e. the source module)

#define DRONE_LINK_CHANNEL_ALL        0

#define DRONE_LINK_PARAM_ALL          0

#define DRONE_LINK_MSG_MAX_PAYLOAD    16

#define DRONE_LINK_MSG_TYPE_UINT8_T   0
#define DRONE_LINK_MSG_TYPE_ADDR      1  // address of parameter e.g. 1>7.8
#define DRONE_LINK_MSG_TYPE_UINT32_T  2
#define DRONE_LINK_MSG_TYPE_FLOAT     3
#define DRONE_LINK_MSG_TYPE_CHAR      4
#define DRONE_LINK_MSG_TYPE_NAME      5   // reply with param name in char format
#define DRONE_LINK_MSG_TYPE_NAMEQUERY 6  // query for the param name
#define DRONE_LINK_MSG_TYPE_QUERY     7  // to query the value of a param, payload should be empty

#define DRONE_LINK_MSG_WRITABLE       0b10000000  // top bit indicates if param is writable

const uint8_t DRONE_LINK_MSG_TYPE_SIZES[16] = {1,4,4,4,1,1,1,1, 1,1,1,1, 1,1,1,1};


struct DRONE_LINK_ADDR {
  uint8_t source; // node id of who originated the packet (e.g. requester for queries, or node if its a published message)
  uint8_t node;  // destination node id for writes, or origin if its a published change
  uint8_t channel;  // message channel ID, e.g. navigation
  uint8_t param;    // e.g. current location
} __packed;

//static_assert(sizeof(DRONE_LINK_ADDR) == 4, "Incorrect Addr size");

union DRONE_LINK_PAYLOAD {
  uint8_t uint8[DRONE_LINK_MSG_MAX_PAYLOAD];
  uint32_t uint32[4];
  float f[4];
  char c[DRONE_LINK_MSG_MAX_PAYLOAD];
  DRONE_LINK_ADDR addr[4];
} __packed;

//static_assert(sizeof(DRONE_LINK_PAYLOAD) == 16, "Incorrect Payload size");

struct DRONE_LINK_MSG {
  uint8_t source; // node id of who originated the packet (e.g. requester for queries, or node if its a published message)
  uint8_t node;  // node id
  uint8_t channel;  // message channel ID, e.g. navigation
  uint8_t param;    // e.g. current location
  uint8_t paramTypeLength;  // e.g. 2x floats for lat/lon
  /*
Upper bit indicates if param is writable
Next 3 bits define type (and implicit data size):

Lower 4 bits define parameter data length (-1) i.e. maps to 1..16
Nominally the length of the data type in bytes, but can be more -
  e.g. a vector3 would be sent as the float data type, length 12 (i.e. 3x 4byte floats)
Strings of up to 16 characters can be sent as char

  */
  //uint8_t payload[DRONE_LINK_MSG_MAX_PAYLOAD];
  DRONE_LINK_PAYLOAD payload;
} __packed;

//static_assert(sizeof(DRONE_LINK_MSG) == 21, "Incorrect Msg size");


class DroneLinkMsg
{
protected:

public:
  DRONE_LINK_MSG _msg;

    DroneLinkMsg() { }

    DroneLinkMsg(DroneLinkMsg *msg) {
      _msg = msg->_msg;
    }

    DroneLinkMsg(DRONE_LINK_MSG *msg) {
      memcpy(&_msg, msg, sizeof(_msg));
    }

    virtual ~DroneLinkMsg() { }

    virtual bool matchParam(uint8_t paramMask) {
      //Serial.print("Mask:");
      //Serial.println(paramMask);
      return (_msg.param == paramMask) || (paramMask == DRONE_LINK_PARAM_ALL);
    }

    // util
    uint8_t packParamLength(boolean w, uint8_t t, uint8_t l) {
      return (w ? DRONE_LINK_MSG_WRITABLE : 0) | ((t & 0x7) << 4) | ((l-1) & 0x0F);
    }

    // setters
    void source(uint8_t v) { _msg.source =v; }
    void node(uint8_t v) { _msg.node = v; }
    void channel(uint8_t v) { _msg.channel = v; }
    void param(uint8_t v) { _msg.param = v; }
    void type(uint8_t v) {
      _msg.paramTypeLength = ((v & 0x7) << 4) | (_msg.paramTypeLength & 0b10001111);
    }
    void length(uint8_t v) {
      _msg.paramTypeLength = (v-1) | (_msg.paramTypeLength & 0xF0);
    }
    void writable(boolean v) {
      if (v) {
        _msg.paramTypeLength = _msg.paramTypeLength | DRONE_LINK_MSG_WRITABLE;
      } else {
        _msg.paramTypeLength = _msg.paramTypeLength & (~DRONE_LINK_MSG_WRITABLE);
      }
    }
    void setUint8_t(uint8_t v) {
      _msg.payload.uint8[0] = v;
    }
    void setFloat(float v) {
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
    uint8_t source() { return _msg.source; }
    uint8_t node() { return _msg.node; }
    uint8_t channel() { return _msg.channel; }
    uint8_t param() { return _msg.param; }
    boolean writable() { return (_msg.paramTypeLength & DRONE_LINK_MSG_WRITABLE) > 0; }
    uint8_t type() {
      return (_msg.paramTypeLength >> 4) & 0x07;
    }
    uint8_t length() {
      return (_msg.paramTypeLength & 0x0F) + 1;
    }
    uint8_t totalSize() {
      return length() + 5;
    }

    // utility

    boolean sameSignature(DroneLinkMsg *msg) {
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

    static void printAddress(DRONE_LINK_ADDR *addr) {
      Serial.print(addr->source);
      Serial.print(':');
      Serial.print(addr->node);
      Serial.print('>');
      Serial.print(addr->channel);
      Serial.print('.');
      Serial.print(addr->param);
    }

    static void printAddress(DRONE_LINK_ADDR *addr, AsyncResponseStream *response) {
      response->print(addr->source);
      response->print(':');
      response->print(addr->node);
      response->print('>');
      response->print(addr->channel);
      response->print('.');
      response->print(addr->param);
    }

    void print() {
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

    static void printPayload(DRONE_LINK_PAYLOAD *payload, uint8_t paramTypeLength) {
      uint8_t ty = (paramTypeLength >> 4) & 0x7;
      uint8_t len = (paramTypeLength & 0xF) + 1;
      switch(ty) {
        case DRONE_LINK_MSG_TYPE_UINT8_T: Serial.print(F("u8")); break;
        case DRONE_LINK_MSG_TYPE_ADDR: Serial.print(F("a")); break;
        case DRONE_LINK_MSG_TYPE_UINT32_T: Serial.print(F("u32")); break;
        case DRONE_LINK_MSG_TYPE_FLOAT: Serial.print(F("f")); break;
        case DRONE_LINK_MSG_TYPE_CHAR: Serial.print(F("c")); break;
        case DRONE_LINK_MSG_TYPE_NAME: Serial.print(F("N")); break;
        case DRONE_LINK_MSG_TYPE_NAMEQUERY: Serial.print(F("AQ")); break;
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

    static void printPayload(DRONE_LINK_PAYLOAD *payload, uint8_t paramTypeLength, AsyncResponseStream *response) {

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

    boolean sameAddress(DRONE_LINK_ADDR *addr) {
      // returns true if channel, param and type match
      return (node() == addr->node &&
             channel() == addr->channel &&
             param() == addr->param);
    }

    // static utils
    static DRONE_LINK_ADDR parseAddress(String addr) {
      // of the form 1>2.3
      DRONE_LINK_ADDR res;
      // defaults
      res.node = 255;
      res.channel = 255;
      res.param = 255;

      // parse
      int gti = addr.indexOf(">");
      int pi = addr.indexOf(".");
      if (gti > -1 && pi > -1) {
        res.node = (addr.substring(0,gti)).toInt();
        res.channel = (addr.substring(gti+1,pi)).toInt();
        res.param = (addr.substring(pi+1)).toInt();
      }

      return res;
    }

};



#endif
