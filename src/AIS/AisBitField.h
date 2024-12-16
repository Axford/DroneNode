
#ifndef AIS_BITFIELD_H
#define AIS_BITFIELD_H

#include "Arduino.h"

#define AIS_BITFIELD_MAX_BITS 168

// class
class AisBitField {
protected:
  uint8_t _numBits;
  char _bits[AIS_BITFIELD_MAX_BITS];
  
public:

  AisBitField();

  void parse(char * msg);

  long getInt(uint8_t start, uint8_t len);
  long getSignedInt(uint8_t start, uint8_t len);

};

#endif

