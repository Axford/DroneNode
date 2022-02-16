#include "DroneMeshMsg.h"

uint8_t getDroneMeshMsgPayloadSize(uint8_t *buffer) {
  return constrain( (buffer[0] & 0b00111111) + 1, 1, DRONE_MESH_MSG_MAX_PAYLOAD_SIZE);
}

void setDroneMeshMsgPayloadSize(uint8_t *buffer, uint8_t size) {
  buffer[0] = (buffer[0] & 0b11000000) | (constrain(size, 1, DRONE_MESH_MSG_MAX_PAYLOAD_SIZE) - 1);
}

uint8_t getDroneMeshMsgTotalSize(uint8_t *buffer) {
  return getDroneMeshMsgPayloadSize(buffer) + sizeof(DRONE_MESH_MSG_HEADER) + 1;
}

uint8_t getDroneMeshMsgPacketType(uint8_t *buffer) {
  return buffer[0] >> 7;
}

void setDroneMeshMsgPacketType(uint8_t *buffer, uint8_t type) {
  buffer[0] = (buffer[0] & 0b01111111) | (type << 7);
}

boolean isDroneMeshMsgAck(uint8_t *buffer) {
  return getDroneMeshMsgPacketType(buffer) > 0;
}

boolean isDroneMeshMsgGuaranteed(uint8_t *buffer) {
  DRONE_MESH_MSG_HEADER* header = (DRONE_MESH_MSG_HEADER*)buffer;
  // check guarantee bit in header
  return (header->typeGuaranteeSize & DRONE_MESH_MSG_GUARANTEED) > 0;
}

uint8_t getDroneMeshMsgTxNode(uint8_t *buffer) {
  return buffer[1];
}

uint8_t getDroneMeshMsgSrcNode(uint8_t *buffer){
  return buffer[2];
}

uint8_t getDroneMeshMsgNextNode(uint8_t *buffer){
  return buffer[3];
}

uint8_t getDroneMeshMsgDestNode(uint8_t *buffer){
  return buffer[4];
}

uint8_t getDroneMeshMsgSeq(uint8_t *buffer){
  return buffer[5];
}

uint8_t getDroneMeshMsgPriority(uint8_t *buffer) {
  return buffer[6] >> 6;
}

void setDroneMeshMsgPriority(uint8_t *buffer, uint8_t priority) {
  buffer[6] = (buffer[6] & 0b00111111) | (priority << 6);
}

uint8_t getDroneMeshMsgPayloadType(uint8_t *buffer) {
  return buffer[6] & 0b111111;
}

void setDroneMeshMsgPayloadType(uint8_t *buffer, uint8_t type) {
  buffer[6] = (buffer[6] & 0b11000000) | type;
}

void setDroneMeshMsgPriorityAndPayloadType(uint8_t *buffer, uint8_t priority, uint8_t type) {
  buffer[6] = priority << 6 | type;
}
