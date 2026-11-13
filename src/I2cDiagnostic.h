#pragma once
#include <stdint.h>

namespace i2cDiagnostic {
enum class Status : uint8_t { Pending, Complete, SelectionFailed, Nack, Timeout, ShortRead, BusError, NoMultiplexer };
constexpr uint8_t firstAddress = 8;
constexpr uint8_t lastAddress = 0x77;

inline bool addressAllowed(unsigned address) {
  return address >= firstAddress && address <= lastAddress;
}

inline bool registerAllowed(unsigned address, unsigned reg, unsigned length) {
  return (address == 0x0d && reg == 0x0d && length == 1) ||
    (address >= 0x40 && address <= 0x43 && (reg == 0xfe || reg == 0xff) && length == 2) ||
    ((address == 0x76) && reg == 0xd0 && length == 1);
}

struct Job {
  uint32_t id = 0;
  uint8_t node = 0, mux = 0;
  bool read = false;
  bool done = false;
  uint8_t channel = 0, address = firstAddress, reg = 0, length = 0, received = 0;
  uint8_t bytes[4] = {};
  Status status[8] = {};
  uint8_t acknowledgements[8][16] = {};

  // Select again on every batch: modules may use a different channel between calls.
  template<class Bus> void step(Bus& bus) {
    if (done) return;
    if (!mux) {
      for (auto& result : status) result = Status::NoMultiplexer;
      done = true;
      return;
    }
    if (!bus.select(channel)) {
      status[channel] = Status::SelectionFailed;
      advance();
      return;
    }
    if (read) {
      status[channel] = bus.read(address, reg, length, bytes, received);
      done = true;
      return;
    }
    for (uint8_t count = 0; count < 4 && address <= lastAddress; ++count, ++address) {
      if (!addressAllowed(address) || address == mux) continue;
      const auto result = bus.ping(address);
      if (result == Status::Complete) acknowledgements[channel][address / 8] |= 1 << (address % 8);
      else if (result != Status::Nack) {
        status[channel] = result;
        advance();
        return;
      }
    }
    if (address > lastAddress) {
      status[channel] = Status::Complete;
      advance();
    }
  }

  bool acknowledged(unsigned bus, unsigned addr) const {
    return acknowledgements[bus][addr / 8] & (1 << (addr % 8));
  }

private:
  void advance() {
    if (read || channel == 7) done = true;
    else {
      ++channel;
      address = firstAddress;
    }
  }
};
}
