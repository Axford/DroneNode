#include "I2cDiagnostic.h"
#include <cassert>
using namespace i2cDiagnostic;

struct FakeBus {
  unsigned channel = 0, reads = 0;
  bool mux = true;
  bool hasMux() { return mux; }
  bool select(unsigned selected) {
    channel = selected;
    return channel != 3;
  }
  Status ping(unsigned address) {
    assert(channel != 3 && addressAllowed(address));
    if (channel == 6 && address == 0x20) return Status::Timeout;
    return (channel == 1 || channel == 5) && address == 0x40 ? Status::Complete : Status::Nack;
  }
  Status read(unsigned address, unsigned reg, unsigned length, uint8_t* data, uint8_t& received) {
    assert(channel != 3 && registerAllowed(address, reg, length));
    ++reads;
    data[0] = 0x32;
    received = 1;
    return Status::ShortRead;
  }
};

int main() {
  FakeBus bus;
  Job scan;
  unsigned ticks = 0;
  while (!scan.done && ticks++ < 256) scan.step(bus);
  assert(scan.done && ticks > 8);
  assert(scan.acknowledged(1, 0x40) && scan.acknowledged(5, 0x40));
  assert(!scan.acknowledged(3, 0x40) && scan.status[3] == Status::SelectionFailed);
  assert(scan.status[6] == Status::Timeout && scan.status[7] == Status::Complete);
  Job read;
  read.read = true;
  read.channel = 3;
  read.address = 0x40;
  read.reg = 0xff;
  read.length = 2;
  auto failed = read;
  failed.step(bus);
  assert(failed.done && bus.reads == 0 && failed.status[3] == Status::SelectionFailed);
  read.channel = 5;
  read.step(bus);
  assert(read.done && bus.reads == 1 && read.received == 1 && read.status[5] == Status::ShortRead);
  bus.mux = false;
  Job absent;
  absent.step(bus);
  assert(absent.done && absent.status[7] == Status::NoMultiplexer);
}
