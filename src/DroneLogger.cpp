
#include "DroneLogger.h"

#include "ArduinoLog.h"
#include "SD.h"
#include <SPI.h>


DroneLogger::DroneLogger() {
  _SDAvailable = false;
  _enabled = true; // enable by default
  _cardSize = 0;
  _cardType = 0;
  _lastFlush = 0;
}


boolean DroneLogger::begin() {
  // ensure pullUo on MISO pin
  // ref: pinMode(23,INPUT_PULLUP); 
  pinMode(23,INPUT_PULLUP); 

  if (!SD.begin(4)) {
    Log.errorln("Card Mount Failed");
    return false;
  }
  _cardType = SD.cardType();

  if(_cardType == CARD_NONE){
    Log.errorln("No SD card attached");
    return false;
  }

  Log.noticeln("SD Card Type: ");
  if(_cardType == CARD_MMC){
    Log.noticeln("MMC");
  } else if(_cardType == CARD_SD){
    Log.noticeln("SDSC");
  } else if(_cardType == CARD_SDHC){
    Log.noticeln("SDHC");
  } else {
    Log.noticeln("UNKNOWN");
  }

  _cardSize = SD.cardSize();
  Log.noticeln("SD Card Size: %f.1 MB\n", (_cardSize/(1024.0f*1024.0f)));
  Log.noticeln("  used: %f.1 MB\n", (SD.usedBytes()/(1024.0f*1024.0f)));
  Log.noticeln("  total: %f.1 MB\n", (SD.totalBytes()/(1024.0f*1024.0f)));

  _SDAvailable = true;

  // create log file if doesn't already exist
  if (!SD.exists("/test.log")) {
    Log.noticeln("Creating log file");
    File f = SD.open("/test.log", FILE_WRITE);
    f.close();
  }

  // start a new log file, ready for appending
  //_file = SD.open("/test.log", FILE_APPEND);
  //if(!_file) {
  //  Log.errorln("Failed to open log for appending");
  //  return false;
  //}

  return true;
}


void DroneLogger::enable() {
  _enabled = true;
}

void DroneLogger::disable() {
  _enabled = false;
}


void DroneLogger::write(uint8_t * buffer, uint32_t len) {
  if (_SDAvailable && _enabled) {
    Log.noticeln("Logging packet: %u bytes", len);

    File f = SD.open("/test.log", FILE_APPEND);
    if (f) {
      /*
      uint32_t bytesWritten = f.write(buffer, len);
      if (bytesWritten != len) {
        Log.errorln("  write fail, only %u bytes written", bytesWritten);
      }
      */
      f.println("test");

      f.close();
    } else {
      Log.errorln("Unable to append to test.log");
    }
  }
  yield();
}

/*
void DroneLogger::write(uint8_t * buffer, uint32_t len) {
  if (_SDAvailable && _file && _enabled) {
    Log.noticeln("Logging packet: %u bytes", len);
    
    uint32_t bytesWritten = _file.write(buffer, len);
    if (bytesWritten != len) {
      Log.errorln("  write fail, only %u bytes written", bytesWritten);
    }

    uint32_t loopTime = millis();
    if (loopTime > _lastFlush + 5000) {
      _lastFlush = loopTime;
      // close and re-open to flush to disk.... as flush() doens't seem to work!
      _file.close();
      delay(1);
      _file = SD.open("/test.log", FILE_APPEND);
    }
  }
}
*/


DroneLogger DroneLog = DroneLogger();