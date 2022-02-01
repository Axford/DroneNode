#include "DroneMeshMsg.h"



uint8_t getDroneMeshMsgPayloadSize(uint8_t *buffer) {
  return (buffer[0] & 0b00111111) + 1;
}


uint8_t getDroneMeshMsgTotalSize(uint8_t *buffer) {
  return getDroneMeshMsgPayloadSize(buffer) + sizeof(DRONE_MESH_MSG_HEADER) + 1;
}


boolean isDroneMeshMsgGuaranteed(uint8_t *buffer) {
  DRONE_MESH_MSG_HEADER* header = (DRONE_MESH_MSG_HEADER*)buffer;
  // check guarantee bit in header
  return (header->modeGuaranteeSize & DRONE_MESH_MSG_GUARANTEED) > 0;
}
