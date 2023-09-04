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

// priorities to be used for mesh packet prioritisation
#define DRONE_LINK_MSG_PRIORITY_LOW       0
#define DRONE_LINK_MSG_PRIORITY_MEDIUM    1
#define DRONE_LINK_MSG_PRIORITY_HIGH      2
#define DRONE_LINK_MSG_PRIORITY_CRITICAL  3


struct DRONE_LINK_ADDR {
  uint8_t source; // node id of who originated the packet (e.g. requester for queries, or node if its a published message)
  uint8_t node;  // destination node id for writes, or origin if its a published change
  uint8_t channel;  // message channel ID, e.g. navigation
  uint8_t paramPriority;    // packed priority (upper 2 bits) and param address (lower 6 bits)
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
  uint8_t paramPriority;    // packed priority (upper 2 bits) and param address (lower 6 bits)
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


uint8_t setDroneLinkMsgPriorityParam(uint8_t pri, uint8_t param);

uint8_t getDroneLinkMsgParam(uint8_t paramPriority);
uint8_t getDroneLinkMsgPriority(uint8_t paramPriority);

class DroneLinkMsg
{
protected:

public:
  DRONE_LINK_MSG _msg;

    DroneLinkMsg() { }

    DroneLinkMsg(DroneLinkMsg *msg);

    DroneLinkMsg(DRONE_LINK_MSG *msg);

    virtual ~DroneLinkMsg();

    virtual bool matchParam(uint8_t paramMask);

    // util
    uint8_t packParamLength(boolean w, uint8_t t, uint8_t l);

    // setters
    void source(uint8_t v);
    void node(uint8_t v);
    void channel(uint8_t v);
    void param(uint8_t v);
    void priority(uint8_t v);
    void type(uint8_t v);
    void length(uint8_t v);
    void writable(boolean v);
    void setUint8_t(uint8_t v);
    void setFloat(float v);

    // getters
    uint8_t source();
    uint8_t node();
    uint8_t channel();
    uint8_t param();
    uint8_t priority();
    boolean writable();
    uint8_t type();
    uint8_t length();
    uint8_t totalSize();

    // utility

    boolean sameSignature(DroneLinkMsg *msg);

    static void printAddress(DRONE_LINK_ADDR *addr);

    static void printAddress(DRONE_LINK_ADDR *addr, AsyncResponseStream *response);

    void print();

    static void printPayload(DRONE_LINK_PAYLOAD *payload, uint8_t paramTypeLength);

    static void printPayload(DRONE_LINK_PAYLOAD *payload, uint8_t paramTypeLength, AsyncResponseStream *response);

    boolean sameAddress(DRONE_LINK_ADDR *addr);

    // static utils
    static DRONE_LINK_ADDR parseAddress(String addr);

};



#endif
