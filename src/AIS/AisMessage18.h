
#ifndef AIS_MESSAGE_18_H
#define AIS_MESSAGE_18_H

#include "Arduino.h"
#include "AisMessage.h"
#include "AisBitField.h"

// class
class AisMessage18: public AisMessage {
protected:
  
public:
  float speedOverGround;
  float lon;
  float lat;
  float courseOverGround;
  float heading;

  AisMessage18();

  void parseFromBitField(AisBitField * bitField);
};

#endif

