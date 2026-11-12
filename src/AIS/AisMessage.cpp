#include "AisMessage.h"

AisMessage::AisMessage() {
  rep = 0;
  mmsi = 0;
  type = 0;
}

void AisMessage::parseFromBitField(AisBitField * bitField) {
  rep = bitField->getInt(6,2);
  mmsi = bitField->getInt(8,30);
}

