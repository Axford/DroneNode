#include "DroneWire.h"

void DroneWire::setup() {
  // configure reset pin and set high to enable the chip
  reset();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
}

void DroneWire::reset() {
  Log.noticeln(F("Reset multiplexer"));
  pinMode(PIN_I2C_RESET, OUTPUT);
  digitalWrite(PIN_I2C_RESET, LOW);
  delay(1);
  //pinMode(PIN_I2C_RESET, INPUT_PULLUP);
  digitalWrite(PIN_I2C_RESET, HIGH);
  delay(1);
}

void DroneWire::selectChannel(uint8_t chan) {
  if (chan> 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << chan);
  Wire.endTransmission();
}

// scan current channel for devices
void DroneWire::scan() {
  for (uint8_t addr = 1; addr<127; addr++) {
      if (addr == TCAADDR) continue;

      //Log.noticeln(F("%d"), addr);

      Wire.beginTransmission(addr);
      byte error = Wire.endTransmission();
      if (error == 0) {
        Log.noticeln(F("Found I2C 0x%x"), addr);
      } else if (error == 4) {
        Log.noticeln(F("Unknown response I2C 0x%x"), addr);
      }

      yield();
    }
}

void DroneWire::scanAll() {
  for (uint8_t i=0; i<8; i++) {
    Log.noticeln(F("Scanning I2C Chan %d..."), i);
    selectChannel(i);
    scan();
    delay(1);
  }
  Log.noticeln(F("Scan complete"));
}
