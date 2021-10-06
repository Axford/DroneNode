#ifndef DRONE_LINK_MSG_H
#define DRONE_LINK_MSG_H

#include "Arduino.h"
#include <ArduinoLog.h>

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

const uint8_t DRONE_LINK_MSG_TYPE_SIZES[16] = {1,2,4,4, 4,8,1,1, 1,1,1,1, 1,1,1,1};

#define DRONE_LINK_MSG_ADDRESS_SUB    0  // this address is a subscription (input)
#define DRONE_LINK_MSG_ADDRESS_PUB    1  // this value is published to address (output)

struct DRONE_LINK_ADDR {
  uint8_t node;  // node id
  uint8_t channel;  // message channel ID, e.g. navigation
  uint8_t param;    // e.g. current location
  uint8_t addrType;   // e.g. DRONE_LINK_MSG_ADDRESS_SUB
};


union DRONE_LINK_PAYLOAD {
  uint8_t uint8[16];
  DRONE_LINK_ADDR addr[4]; 
  uint32_t uint32[4];
  float f[4];
  char c[16];
};


struct DRONE_LINK_MSG {
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
};

class DroneLinkMsg
{
protected:

public:
  DRONE_LINK_MSG _msg;

    DroneLinkMsg() { }

    DroneLinkMsg(DroneLinkMsg *msg) {
      _msg = msg->_msg;
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

    // utility

    boolean sameSignature(DroneLinkMsg *msg) {
      // returns true if channel, param and type match
      return (node() == msg->node()) &&
             (channel() == msg->channel()) &&
             (param() == msg->param()) &&
             (type() == msg->type());
    }

    void print() {
      // TODO: convert to use Log.notice xx
      Serial.print(node());
      Serial.print('.');
      Serial.print(channel());
      Serial.print('.');
      Serial.print(param());
      Serial.print(' ');
      switch(type()) {
        case DRONE_LINK_MSG_TYPE_UINT8_T: Serial.print(F("uint8_t")); break;
        case DRONE_LINK_MSG_TYPE_ADDR: Serial.print(F("addr")); break;
        case DRONE_LINK_MSG_TYPE_UINT32_T: Serial.print(F("uint32_t")); break;
        case DRONE_LINK_MSG_TYPE_FLOAT: Serial.print(F("Float")); break;
        case DRONE_LINK_MSG_TYPE_CHAR: Serial.print(F("Char")); break;
        case DRONE_LINK_MSG_TYPE_QUERY: Serial.print(F("Query")); break;
      }
      if (type() != DRONE_LINK_MSG_TYPE_QUERY) {
        Serial.print('[');
        Serial.print(length());
        Serial.print(']');
        Serial.print('=');
      }

      switch(type()) {
        case DRONE_LINK_MSG_TYPE_UINT8_T: Serial.print(_msg.payload.uint8[0], HEX); break;
        //case DRONE_LINK_MSG_TYPE_UINT32_T: Serial.print(F("uint32_t")); break;
        //case DRONE_LINK_MSG_TYPE_FLOAT: Serial.print(F("Float")); break;
        case DRONE_LINK_MSG_TYPE_CHAR:
          for (uint8_t i=0; i<length(); i++) {
            Serial.print(char(_msg.payload.c[i]));
          }
          break;
        //case DRONE_LINK_MSG_TYPE_QUERY: Serial.print(F("Query")); break;
      }
      Serial.print('\n');
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
