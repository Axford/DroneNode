#include "DroneWire.h"

uint8_t _lastChan;
uint8_t _TCAADDR = TCAADDR_V2;

void DroneWire::setup() {
  // configure reset pin and set high to enable the chip
  reset();

  _lastChan = 10;

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeout(100);

  _TCAADDR = 0;

  // test if we are on a v1 or v2 motherboard
  Wire.beginTransmission(TCAADDR_V1);
  byte error = Wire.endTransmission();
  if (error == 0) {
    Log.noticeln("[DW.s] Found v1 Multiplexer");
    _TCAADDR = TCAADDR_V1;
  }

  Wire.beginTransmission(TCAADDR_V2);
  error = Wire.endTransmission();
  if (error == 0) {
    Log.noticeln("[DW.s] Found v2+ Multiplexer");
    _TCAADDR = TCAADDR_V2;
  }

  if (_TCAADDR == 0) {
    Log.errorln("[DW.s] Multiplexer not found");
  }
}


uint8_t DroneWire::getMultiplexerVersion() {
  if (_TCAADDR == TCAADDR_V1) return 1;
  if (_TCAADDR == TCAADDR_V2) return 2;
  return 0;
}


void DroneWire::reset() {
  Log.noticeln(F("[DW.r] Reset multiplexer"));
  pinMode(PIN_I2C_RESET, OUTPUT);
  digitalWrite(PIN_I2C_RESET, LOW);
  delay(1);
  //pinMode(PIN_I2C_RESET, INPUT_PULLUP);
  digitalWrite(PIN_I2C_RESET, HIGH);
  delay(1);

  _lastChan = 8;  // to ensure the correct channel is reselected

  // check comms to multiplexer
  /*
  Wire.beginTransmission(_TCAADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Log.noticeln("[DW.r] Multiplexer not responding");
  }
  */
}

void DroneWire::selectChannel(uint8_t chan) {
  if (chan> 7 || chan == _lastChan) return;

  //Log.noticeln("[DW.sC] %u", chan);
  Wire.beginTransmission(_TCAADDR);
  Wire.write(1 << chan);
  Wire.endTransmission();

  _lastChan = chan;
}


// scan a specific address on current channel, return true if found
boolean DroneWire::scanAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  byte error = Wire.endTransmission();
  if (error == 0) {
    return true;
  } else if (error == 4) {
    // unknown I2C response
  }
  return false;
}


// scan current channel for devices
void DroneWire::scan() {
  for (uint8_t addr = 1; addr<127; addr++) {
      if (addr == _TCAADDR) continue;

      //Log.noticeln(F("%d"), addr);

      Wire.beginTransmission(addr);
      byte error = Wire.endTransmission();
      if (error == 0) {
        Log.noticeln(F("  Found I2C 0x%x"), addr);
      } else if (error == 4) {
        Log.noticeln(F("  Unknown response I2C 0x%x"), addr);
      }

      yield();
    }
}

void DroneWire::scanAll() {
  for (uint8_t i=0; i<8; i++) {
    Log.noticeln(F("[DW.sA] Scanning I2C Chan %d..."), i);
    selectChannel(i);
    scan();
    delay(1);
  }
  Log.noticeln(F("[DW.sA] Scan complete"));
}
