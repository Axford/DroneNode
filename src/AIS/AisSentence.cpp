#include "../DroneConsole.h"
#include "AisSentence.h"
#include "Arduino.h"

AisSentence::AisSentence() {

}


boolean AisSentence::parse(char * msg, uint8_t len) {

  DroneConsole.print(" NMEA: ");
  DroneConsole.write(msg, len);
  DroneConsole.println();
  // !AIVDM,1,1,0,A,B3P8C0h007tmj8WGIPH><03WCP06,0*6A

  // reset
  _channel = 0;
  _aisType = 0;

  // see if msg begins with !AIVDM,1,1,
  if (len < 48 || strncmp(msg, "!AIVDM,1,1,", 11) !=0 ) {
    DroneConsole.println("  Not AIS");
    return false;
  }

  uint8_t i=1;
  uint32_t checksum = 0;
  // calc checksum... no point parsing a junk message
  while (msg[i] != 0 && msg[i] != '*') {
    checksum = checksum ^ msg[i];
    i++;
  }
  String checksumHex = String(checksum, HEX);
  checksumHex.toUpperCase();
  if (checksumHex.length() == 1) checksumHex = '0' + checksumHex;

  // now compare checksums
  if (msg[i] != '*') {
    DroneConsole.println("  Missing checksum");
    return false;
  }
  i++;

  if (msg[i] != checksumHex[0] && msg[i+1] != checksumHex[1]) {
    DroneConsole.println("  Failed Checksum");
    return false;
  }

  // start parsing from partId, part 3
  i=11;
  uint8_t commas = 0;
  uint8_t bufLen = 0;
  char buf[AIS_SENTENCE_MAX_PART_LENGTH];
  while (msg[i] != 0 && msg[i] != '*') {
    switch (msg[i]) {
      case '!': break;
      case '$': break;
      case ',': 
                // null terminate buffer
                buf[bufLen] = 0;
                // handle parts... starting at partId
                if (commas == 1) {
                  // channel
                  _channel = buf[0];
                } else if (commas == 2) {
                  // payload
                  // check its expected length = 28
                  if (bufLen != 28) return false;

                  // parse AIS payload
                  //Serial.print("  Payload: ");
                  //Serial.println(buf);
                  
                  _bitField.parse(buf);

                  // get the message type
                  _aisType = _bitField.getInt(0,6);
                  DroneConsole.print("  Type: ");
                  DroneConsole.println(_aisType);

                  if (_aisType == 18) {
                    // decode
                    _m18.parseFromBitField(&_bitField);

                    // print
                    DroneConsole.print("  MMSI: ");
                    DroneConsole.println(_m18.mmsi);
                    DroneConsole.print("  Lat: ");
                    DroneConsole.println(_m18.lat);
                    DroneConsole.print("  Lon: ");
                    DroneConsole.println(_m18.lon);
                    DroneConsole.print("  SOG: ");
                    DroneConsole.println(_m18.speedOverGround);
                    DroneConsole.print("  COG: ");
                    DroneConsole.println(_m18.courseOverGround);
                    DroneConsole.print("  H: ");
                    DroneConsole.println(_m18.heading);

                    return true;
                  }
                }
                commas++; 
                bufLen=0; 
                break;
      default: 
        // add to buffer
        buf[bufLen] = msg[i];
        if (bufLen < AIS_SENTENCE_MAX_PART_LENGTH-1) bufLen++;
    }

    i++;
  }

  return false;
}


uint8_t AisSentence::getAisType() {
  return _aisType;
}