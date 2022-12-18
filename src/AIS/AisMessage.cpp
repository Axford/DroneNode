#include "AisMessage.h"

AisMessage::AisMessage() {
  
}

void AisMessage::parseFromBitField(AisBitField * bitField) {
  rep = bitField->getInt(6,2);
  mmsi = bitField->getInt(8,30);
}

