
#ifndef AIS_SENTENCE_H
#define AIS_SENTENCE_H

#include "Arduino.h"
#include "AisBitField.h"
#include "AisMessage18.h"

#define AIS_SENTENCE_MAX_PART_LENGTH 32

// class
class AisSentence {
protected:
  char _channel;
  AisBitField _bitField;
  uint8_t _aisType;
public:
  AisMessage18 _m18;

  AisSentence();

  boolean parse(char * msg, uint8_t len);

  uint8_t getAisType();

};

#endif

