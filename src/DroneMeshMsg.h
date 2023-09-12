#ifndef DRONE_MESH_MSG_H
#define DRONE_MESH_MSG_H

#include "Arduino.h"

// DroneMeshMsg is header + payload + CRC8
struct DRONE_MESH_MSG_HEADER {
  uint8_t typeGuaranteeSize;  // packet type, guarantee, size
  uint8_t txNode;
  uint8_t srcNode;
  uint8_t nextNode; // for unicast
  uint8_t destNode;
  uint8_t seq;
  uint8_t priorityType;  // payload priority (2bit) and type (6bit)
} __packed;  // 7 bytes


// Packet type
#define DRONE_MESH_MSG_SEND             0
#define DRONE_MESH_MSG_ACK              1  // ACK

// guarantee
#define DRONE_MESH_MSG_NOT_GUARANTEED   0
#define DRONE_MESH_MSG_GUARANTEED       0b01000000

// Payload types
// -------------------------------------------------------------------------
#define DRONE_MESH_MSG_TYPE_HELLO                    0
#define DRONE_MESH_MSG_TYPE_SUBSCRIPTION_REQUEST     1
#define DRONE_MESH_MSG_TYPE_SUBSCRIPTION_RESPONSE    2

#define DRONE_MESH_MSG_TYPE_TRACEROUTE_REQUEST       3
#define DRONE_MESH_MSG_TYPE_TRACEROUTE_RESPONSE      4

#define DRONE_MESH_MSG_TYPE_ROUTEENTRY_REQUEST       5
#define DRONE_MESH_MSG_TYPE_ROUTEENTRY_RESPONSE      6

#define DRONE_MESH_MSG_TYPE_DRONELINKMSG             7

#define DRONE_MESH_MSG_TYPE_ROUTER_REQUEST           20
#define DRONE_MESH_MSG_TYPE_ROUTER_RESPONSE          21

#define DRONE_MESH_MSG_TYPE_LINK_CHECK_REQUEST       26

// batch firmware messages
#define DRONE_MESH_MSG_TYPE_FIRMWARE_START_REQUEST   22
#define DRONE_MESH_MSG_TYPE_FIRMWARE_START_RESPONSE  23
#define DRONE_MESH_MSG_TYPE_FIRMWARE_WRITE           24
#define DRONE_MESH_MSG_TYPE_FIRMWARE_REWIND          25

// filesystem messages
/*
v1
#define DRONE_MESH_MSG_TYPE_FS_FILE_REQUEST          10
#define DRONE_MESH_MSG_TYPE_FS_FILE_RESPONSE         11
#define DRONE_MESH_MSG_TYPE_FS_RESIZE_REQUEST        12
#define DRONE_MESH_MSG_TYPE_FS_RESIZE_RESPONSE       13
#define DRONE_MESH_MSG_TYPE_FS_READ_REQUEST          14
#define DRONE_MESH_MSG_TYPE_FS_READ_RESPONSE         15
#define DRONE_MESH_MSG_TYPE_FS_WRITE_REQUEST         16
#define DRONE_MESH_MSG_TYPE_FS_WRITE_RESPONSE        17
*/
// v2
#define DRONE_MESH_MSG_TYPE_FS_FILE_REQUEST          10
#define DRONE_MESH_MSG_TYPE_FS_FILE_RESPONSE         11
#define DRONE_MESH_MSG_TYPE_FS_MANAGE_REQUEST        12
#define DRONE_MESH_MSG_TYPE_FS_MANAGE_RESPONSE       13
#define DRONE_MESH_MSG_TYPE_FS_READ_REQUEST          14
#define DRONE_MESH_MSG_TYPE_FS_READ_RESPONSE         15
#define DRONE_MESH_MSG_TYPE_FS_WRITE_REQUEST         16
#define DRONE_MESH_MSG_TYPE_FS_WRITE_RESPONSE        17

// -------------------------------------------------------------------------

// Priorities
// -------------------------------------------------------------------------
#define DRONE_MESH_MSG_PRIORITY_LOW        0
#define DRONE_MESH_MSG_PRIORITY_MEDIUM     1
#define DRONE_MESH_MSG_PRIORITY_HIGH       2
#define DRONE_MESH_MSG_PRIORITY_CRITICAL   3
// -------------------------------------------------------------------------

struct DRONE_MESH_MSG_HELLO {
  DRONE_MESH_MSG_HEADER header;
  uint8_t metric;
  uint32_t uptime;
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_SUBCSRIPTION_REQUEST {
  DRONE_MESH_MSG_HEADER header;
  uint8_t channel;
  uint8_t param;
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_TRACEROUTE_REQUEST {
  DRONE_MESH_MSG_HEADER header;
  uint8_t dummyNode;
  uint8_t dummyMetric;
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_ROUTEENTRY_REQUEST {
  DRONE_MESH_MSG_HEADER header;
  uint8_t node;
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_ROUTEENTRY_RESPONSE {
  DRONE_MESH_MSG_HEADER header;
  uint8_t src;
  uint8_t node;
  uint8_t seq;
  uint8_t metric;
  uint8_t interfaceType;
  uint8_t nextHop;
  uint32_t age;
  uint32_t uptime;
  uint8_t avgAttempts;  // encoded as value x 10 (i.e. in range 0..100)
  uint8_t avgAckTime;  // in ms, rounded to nearest ms
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_ROUTER_REQUEST {
  DRONE_MESH_MSG_HEADER header;
  uint8_t padding;
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_ROUTER_RESPONSE {
  DRONE_MESH_MSG_HEADER header;
  uint8_t txQueueSize;
  uint8_t txQueueActive; // ready or waiting
  uint32_t kicked;
  uint32_t choked;
  uint8_t chokeRate; // x10 and rounded.  e.g. 0.2 becomes 20
  uint8_t kickRate;  // x10 and rounded
  uint8_t utilisation;  // x100 and rounded .. i.e. utilisation %
  uint8_t crc;
} __packed;  // header(7) + 6 + 8 = 21

struct DRONE_MESH_MSG_LINK_CHECK_REQUEST {
  DRONE_MESH_MSG_HEADER header;
  uint8_t padding;
  uint8_t crc;
} __packed;

// ----------------------------------------------------------------------------
// firmware messages
// ----------------------------------------------------------------------------
struct DRONE_MESH_MSG_FIRMWARE_START_REQUEST {
  DRONE_MESH_MSG_HEADER header;
  uint32_t size;
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_FIRMWARE_START_RESPONSE {
  DRONE_MESH_MSG_HEADER header;
  uint8_t status;
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_FIRMWARE_WRITE {
  DRONE_MESH_MSG_HEADER header;
  uint32_t offset;
  uint8_t data[44];
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_FIRMWARE_REWIND {
  DRONE_MESH_MSG_HEADER header;
  uint32_t offset;
  uint8_t crc;
} __packed;


// ----------------------------------------------------------------------------
// filesystem
// ----------------------------------------------------------------------------

#define DRONE_MESH_MSG_FS_MAX_PATH_SIZE       24  // inc null termination
#define DRONE_MESH_MSG_FS_DATA_SIZE           32

#define DRONE_MESH_MSG_FS_FLAG_PATH_INFO      0  // request entry info as specified by path
#define DRONE_MESH_MSG_FS_FLAG_INDEX_INFO     1  // request entry info as specified by index
#define DRONE_MESH_MSG_FS_FLAG_DELETE         2  // delete file/directory by path


#define DRONE_MESH_MSG_FS_FLAG_SUCCESS        0
#define DRONE_MESH_MSG_FS_FLAG_DIRECTORY      4
#define DRONE_MESH_MSG_FS_FLAG_FILE           5
#define DRONE_MESH_MSG_FS_FLAG_NOT_FOUND      6
#define DRONE_MESH_MSG_FS_FLAG_ERROR          7

// Manage Request flags
#define DRONE_MESH_MSG_FS_FLAG_START          0
#define DRONE_MESH_MSG_FS_FLAG_QUERY          1
#define DRONE_MESH_MSG_FS_FLAG_SAVE           2

// Manage Response flags
#define DRONE_MESH_MSG_FS_FLAG_WIP            8  // transfer in progress
#define DRONE_MESH_MSG_FS_FLAG_READY          9  // all blocks written, ready to save to disk


struct DRONE_MESH_MSG_FS_FILE_REQUEST {
  DRONE_MESH_MSG_HEADER header;
  uint8_t flags;  // DRONE_MESH_MSG_FS_FLAG_PATH_INFO | DRONE_MESH_MSG_FS_FLAG_INDEX_INFO | DRONE_MESH_MSG_FS_FLAG_DELETE
  uint8_t id; // index of file in directory (subject to flags)
  uint8_t path[DRONE_MESH_MSG_FS_MAX_PATH_SIZE];  // null terminated
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_FS_FILE_RESPONSE {
  DRONE_MESH_MSG_HEADER header;
  uint8_t flags;
  uint8_t id; // unique numeric id of entry
  uint32_t size;  // filesize of number of directory entries
  uint8_t path[DRONE_MESH_MSG_FS_MAX_PATH_SIZE];  // null terminated
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_FS_READ_REQUEST {
  DRONE_MESH_MSG_HEADER header;
  uint8_t id; // id of file in directory
  uint32_t offset;
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_FS_READ_RESPONSE {
  DRONE_MESH_MSG_HEADER header;
  uint8_t id; // id of file in directory
  uint32_t offset;
  uint8_t size;
  uint8_t data[DRONE_MESH_MSG_FS_DATA_SIZE];
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_FS_MANAGE_REQUEST {
  DRONE_MESH_MSG_HEADER header;
  uint8_t flags;
  uint32_t size;
  uint8_t path[DRONE_MESH_MSG_FS_MAX_PATH_SIZE];  // null terminated
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_FS_MANAGE_RESPONSE {
  DRONE_MESH_MSG_HEADER header;
  uint8_t flags;
  uint8_t status;
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_FS_WRITE_REQUEST {
  DRONE_MESH_MSG_HEADER header;
  uint32_t offset;
  uint8_t size;
  uint8_t data[DRONE_MESH_MSG_FS_DATA_SIZE];
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_FS_WRITE_RESPONSE {
  DRONE_MESH_MSG_HEADER header;
  uint8_t flags;
  uint32_t offset;
  uint8_t crc;
} __packed;


// ----------------------------------------------------------------------------
#define DRONE_MESH_MSG_MAX_PAYLOAD_SIZE   48 // bytes - to keep within RFM69 transmit size limit
#define DRONE_MESH_MSG_MAX_PACKET_SIZE    (sizeof(DRONE_MESH_MSG_HEADER) + DRONE_MESH_MSG_MAX_PAYLOAD_SIZE + 1 )  // header + payload + CRC

// interface type codes
#define DRONE_MESH_INTERFACE_TYPE_UDP        0
#define DRONE_MESH_INTERFACE_TYPE_RFM69      1
#define DRONE_MESH_INTERFACE_TYPE_SERIAL     2  // serial interface, optionally via transparent telemetry radio
#define DRONE_MESH_INTERFACE_TYPE_IRIDIUM    3


// forward decl
class NetworkInterfaceModule;

// message buffer used in transmit queue
struct DRONE_MESH_MSG_BUFFER {
  uint8_t data[DRONE_MESH_MSG_MAX_PACKET_SIZE];
  uint8_t state;
  NetworkInterfaceModule *interface;  // which interface is responsible for this buffer (if non-empty)
  uint8_t attempts;  // how many attempts have we made to send this
  uint32_t created;  // time packet was created
  uint32_t sent;  // time packet was successfully sent
} __packed;

// buffer states
#define DRONE_MESH_MSG_BUFFER_STATE_EMPTY      0   // empty (or already sent)
#define DRONE_MESH_MSG_BUFFER_STATE_READY      1   // ready to send
#define DRONE_MESH_MSG_BUFFER_STATE_WAITING    2   // waiting for Ack


uint8_t getDroneMeshMsgPayloadSize(uint8_t *buffer);
void setDroneMeshMsgPayloadSize(uint8_t *buffer, uint8_t size);
uint8_t getDroneMeshMsgTotalSize(uint8_t *buffer);

uint8_t getDroneMeshMsgPacketType(uint8_t *buffer);
void setDroneMeshMsgPacketType(uint8_t *buffer, uint8_t type);
boolean isDroneMeshMsgAck(uint8_t *buffer);
boolean isDroneMeshMsgGuaranteed(uint8_t *buffer);

uint8_t getDroneMeshMsgTxNode(uint8_t *buffer);
uint8_t getDroneMeshMsgSrcNode(uint8_t *buffer);
uint8_t getDroneMeshMsgNextNode(uint8_t *buffer);
uint8_t getDroneMeshMsgDestNode(uint8_t *buffer);
uint8_t getDroneMeshMsgSeq(uint8_t *buffer);

uint8_t getDroneMeshMsgPriority(uint8_t *buffer);
void setDroneMeshMsgPriority(uint8_t *buffer, uint8_t priority);

uint8_t getDroneMeshMsgPayloadType(uint8_t *buffer);
void setDroneMeshMsgPayloadType(uint8_t *buffer, uint8_t type);

void setDroneMeshMsgPriorityAndPayloadType(uint8_t *buffer, uint8_t priority, uint8_t type);



#endif
