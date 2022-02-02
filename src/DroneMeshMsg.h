#ifndef DRONE_MESH_MSG_H
#define DRONE_MESH_MSG_H

#include "Arduino.h"

// DroneMeshMsg is header + payload + CRC8
struct DRONE_MESH_MSG_HEADER {
  uint8_t modeGuaranteeSize;  // packed mode, guarantee, size
  uint8_t txNode;
  uint8_t srcNode;
  uint8_t nextNode; // for unicast
  uint8_t destNode;
  uint8_t seq;
  uint8_t typeDir;  // packed type + direction
} __packed;  // 7 bytes

struct DRONE_MESH_MSG_HELLO {
  DRONE_MESH_MSG_HEADER header;
  uint8_t metric;
  uint8_t crc;
} __packed;

struct DRONE_MESH_MSG_SUBCSRIPTION {
  DRONE_MESH_MSG_HEADER header;
  uint8_t channel;
  uint8_t param;
  uint8_t crc;
} __packed;

#define DRONE_MESH_MSG_MAX_PAYLOAD_SIZE   48 // bytes - to keep within RFM69 transmit size limit
#define DRONE_MESH_MSG_MAX_PACKET_SIZE    (6 + DRONE_MESH_MSG_MAX_PAYLOAD_SIZE + 1 )  // header + payload + CRC

struct DRONE_MESH_MSG_BUFFER {
  uint8_t data[DRONE_MESH_MSG_MAX_PACKET_SIZE];
  uint8_t state;
} __packed;

// buffer states
#define DRONE_MESH_MSG_BUFFER_STATE_EMPTY      0   // empty (or already sent)
#define DRONE_MESH_MSG_BUFFER_STATE_READY      1   // ready to send
#define DRONE_MESH_MSG_BUFFER_STATE_WAITING    2   // waiting for Ack


#define DRONE_MESH_MSG_MODE_UNICAST     0
#define DRONE_MESH_MSG_MODE_MULTICAST   0b10000000

#define DRONE_MESH_MSG_NOT_GUARANTEED   0
#define DRONE_MESH_MSG_GUARANTEED       0b01000000

#define DRONE_MESH_MSG_TYPE_HELLO          (0 << 1)
#define DRONE_MESH_MSG_TYPE_SUBSCRIPTION   (1 << 1)
#define DRONE_MESH_MSG_TYPE_TRACEROUTE     (2 << 1)
#define DRONE_MESH_MSG_TYPE_ROUTINGENTRY   (3 << 1)
#define DRONE_MESH_MSG_TYPE_DRONELINKMSG   (4 << 1)

#define DRONE_MESH_MSG_REQUEST          0
#define DRONE_MESH_MSG_RESPONSE         1

uint8_t getDroneMeshMsgMode(uint8_t *buffer);

uint8_t getDroneMeshMsgPayloadSize(uint8_t *buffer);
uint8_t getDroneMeshMsgTotalSize(uint8_t *buffer);

boolean isDroneMeshMsgGuaranteed(uint8_t *buffer);

uint8_t getDroneMeshMsgTxNode(uint8_t *buffer);
uint8_t getDroneMeshMsgSrcNode(uint8_t *buffer);
uint8_t getDroneMeshMsgNextNode(uint8_t *buffer);
uint8_t getDroneMeshMsgDestNode(uint8_t *buffer);
uint8_t getDroneMeshMsgSeq(uint8_t *buffer);
uint8_t getDroneMeshMsgType(uint8_t *buffer);
uint8_t getDroneMeshMsgDirection(uint8_t *buffer);

#endif
