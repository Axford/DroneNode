#include "AisBitField.h"

AisBitField::AisBitField() {
  _numBits = 0;

}


void AisBitField::parse(char * msg) {
  _numBits = 0;
  uint8_t i=0;
  while (msg[i] != 0) {
    uint8_t v = (uint8_t)msg[i] - 48;
    if (v > 40) v -= 8;

    // bit shift through v and store in _bits
    for (uint8_t b=0; b<6; b++) {
      _bits[_numBits+(5-b)] = (v & 1) > 0 ? '1' : '0';
      v = v >> 1;
    }

    _numBits += 6;
    i++;
  }

  //Serial.write(_bits, _numBits);
  //Serial.println();
}


long AisBitField::getInt(uint8_t start, uint8_t len) {
  long value = 0;
  for (int i=0; i< len; i++)  
  {
    value *= 2; // double the result so far
    if (_bits[start + i] == '1') value++;  //add 1 if needed
  }
  return value;
}

long AisBitField::getSignedInt(uint8_t start, uint8_t len) {
  long v = getInt(start, len);
 
  // Convert to signed integer
  long mask = 1 << (len -1);
  long masked = v & mask;
  if ( masked != 0) {
    v -= 1 << len;
  }

  return v;
}