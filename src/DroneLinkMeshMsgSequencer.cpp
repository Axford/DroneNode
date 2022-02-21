#include "DroneLinkMeshMsgSequencer.h"


DroneLinkMeshMsgSequencer::DroneLinkMeshMsgSequencer() {
  clear();
}


void DroneLinkMeshMsgSequencer::clear() {
  for (uint8_t i=0; i<32; i++) {
    _bitmask[i] = 0;
  }
}


boolean DroneLinkMeshMsgSequencer::isDuplicate(uint8_t v) {

  // calc byte index
  uint8_t index = v >> 3;  // divide by 8
  uint8_t bm = 1 << (v & 0b111);  // mask for this bit in associated byte ... lower 3 bits

  // clear opposite semi-circle...  circle is 32 bytes round, so opposite half starts at +8 from current index
  uint8_t p = index + 8;
  if (p > 31) p -= 32;
  for (uint8_t i = 0; i<16; i++) {
    _bitmask[p] = 0;
    p++;
    if (p > 31) p =0;
  }

  // check bit for v
  if ((_bitmask[index] & bm) > 0) {
    // already set
    return true;
  }

  // set bit for v
  _bitmask[index] |= bm;

  return false;
}
