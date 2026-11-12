
#ifndef AIS_MESSAGE_H
#define AIS_MESSAGE_H

#include "Arduino.h"
#include "AisBitField.h"

// class
class AisMessage {
protected:
  
  
public:
  uint8_t type;
  uint8_t rep;
  uint32_t mmsi;

  AisMessage();

  virtual void parseFromBitField(AisBitField * bitField);
};

#endif

