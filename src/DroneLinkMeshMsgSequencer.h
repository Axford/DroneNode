/*

DroneLinkMeshMsgSequencer

Manages a 256-bit circular state buffer, used to avoid duplicative sequencing
of Mesh Hello packets and/or guaranteed packets


*/
#ifndef DRONE_LINK_MESH_MSG_SEQUENCER_H
#define DRONE_LINK_MESH_MSG_SEQUENCER_H

#include "Arduino.h"

class DroneLinkMeshMsgSequencer {
protected:
  uint8_t _bitmask[32];

public:
  DroneLinkMeshMsgSequencer();

  void clear();

  boolean isDuplicate(uint8_t v);

};

#endif
