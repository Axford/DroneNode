#include "DroneMeshMsg.h"

uint8_t getDroneMeshMsgMode(uint8_t *buffer) {
  return buffer[0] & 0b10000000;
}

uint8_t getDroneMeshMsgPayloadSize(uint8_t *buffer) {
  return constrain( (buffer[0] & 0b00111111) + 1, 0, DRONE_MESH_MSG_MAX_PAYLOAD_SIZE);
}

void setDroneMeshMsgPayloadSize(uint8_t *buffer, uint8_t size) {
  buffer[0] = (buffer[0] & 0b11000000) | (constrain(size, 1, DRONE_MESH_MSG_MAX_PAYLOAD_SIZE)-1);
}


uint8_t getDroneMeshMsgTotalSize(uint8_t *buffer) {
  return getDroneMeshMsgPayloadSize(buffer) + sizeof(DRONE_MESH_MSG_HEADER) + 1;
}


boolean isDroneMeshMsgGuaranteed(uint8_t *buffer) {
  DRONE_MESH_MSG_HEADER* header = (DRONE_MESH_MSG_HEADER*)buffer;
  // check guarantee bit in header
  return (header->modeGuaranteeSize & DRONE_MESH_MSG_GUARANTEED) > 0;
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

uint8_t getDroneMeshMsgType(uint8_t *buffer) {
  return buffer[6] & 0b11111110;
}

uint8_t getDroneMeshMsgDirection(uint8_t *buffer) {
  return buffer[6] & 0b1;
}
