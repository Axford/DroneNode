#include "droneModules/ProbeSerial.h"

#include <assert.h>
#include <stdint.h>

namespace {
constexpr uint32_t baud = 9600;

struct Signal {
  probe::SerialDecoder decoder;
  uint32_t origin = 0xfffff000;
  uint32_t bit = 10;
  bool level = true;

  Signal() { decoder.reset(baud, origin, true); }

  void put(bool next) {
    if (next != level) decoder.edge(time(), next);
    level = next;
    ++bit;
  }

  uint32_t time() const { return origin + bit * 1000000 / baud; }

  void send(uint8_t value, bool stop = true) {
    put(false);
    for (uint8_t b = 0; b < 8; ++b) put((value >> b) & 1);
    put(stop);
  }
};
}

int main() {
  Signal stream;
  for (uint8_t i = 0; i < 20; ++i) stream.send(i);
  stream.decoder.advance(stream.time());
  uint8_t bytes[15];
  assert(stream.decoder.copyTail(bytes) == 15);
  for (uint8_t i = 0; i < 15; ++i) assert(bytes[i] == i + 5);

  Signal badStop;
  badStop.send(0, false);
  badStop.decoder.advance(badStop.time());
  assert(badStop.decoder.copyTail(bytes) == 0);

  Signal midFrame;
  midFrame.bit = 0;
  for (int i = 0; i < 4; ++i) midFrame.send(0x33);
  midFrame.decoder.advance(midFrame.time());
  assert(midFrame.decoder.copyTail(bytes) == 0);
  midFrame.bit += 10;
  midFrame.send(0xa5);
  midFrame.decoder.advance(midFrame.time());
  assert(midFrame.decoder.copyTail(bytes) == 1 && bytes[0] == 0xa5);

  Signal incomplete;
  incomplete.send(0xff);
  incomplete.decoder.advance(incomplete.time() - 200);
  assert(incomplete.decoder.copyTail(bytes) == 0);
  incomplete.decoder.advance(incomplete.time());
  assert(incomplete.decoder.copyTail(bytes) == 1 && bytes[0] == 0xff);
}
