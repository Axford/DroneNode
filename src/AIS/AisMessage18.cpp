#include "AisMessage18.h"

AisMessage18::AisMessage18() {
  type = 18;

  speedOverGround =0;
  lon=0;
  lat=0;
  courseOverGround=0;
  heading=0;
}

void AisMessage18::parseFromBitField(AisBitField * bitField) {
  AisMessage::parseFromBitField(bitField);

  speedOverGround = bitField->getInt(46,10) / 10.0;
  courseOverGround = bitField->getInt(112, 12) / 10.0;
  heading = bitField->getInt(124, 9);
  lon = bitField->getSignedInt(57, 28) / 600000.0;
  lat = bitField->getSignedInt(85, 27) / 600000.0;
}