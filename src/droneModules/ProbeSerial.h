#ifndef DRONE_PROBE_SERIAL_H
#define DRONE_PROBE_SERIAL_H

#include <stdint.h>

namespace probe {

// Decode a non-inverted 8N1 signal from timestamped GPIO changes.
class SerialDecoder {
public:
  void reset(uint32_t baud, uint32_t time, bool high) {
    _baud = baud;
    _size = _next = _bit = 0;
    _receiving = _synchronized = false;
    _high = high;
    _highSince = time;
  }

  void edge(uint32_t time, bool high) {
    advance(time);
    if (high && !_high) _highSince = time;
    if (!high && _high && time - _highSince >= 9000000UL / _baud) _synchronized = true;
    _high = high;
    if (_synchronized && !_receiving && !high) {
      _receiving = true;
      _start = time;
      _bit = 0;
      _byte = 0;
    }
  }

  void advance(uint32_t time) {
    if (!_receiving) return;
    // Compare against the centres of the start bit, eight data bits and stop bit.
    const uint64_t elapsed = static_cast<uint64_t>(time - _start) * _baud * 2;
    while (_receiving && elapsed > (2 * _bit + 1) * 1000000UL) {
      if (_bit == 0 && _high) {
        _receiving = false;
      } else if (_bit == 9) {
        if (_high) {
          _bytes[_next] = _byte;
          _next = (_next + 1) % capacity;
          if (_size < capacity) ++_size;
        } else {
          _synchronized = false;
        }
        _receiving = false;
      } else if (_bit && _high) {
        _byte |= 1 << (_bit - 1);
      }
      ++_bit;
    }
  }

  uint8_t copyTail(uint8_t* output) const {
    for (uint8_t i = 0; i < _size; ++i) {
      output[i] = _bytes[(_next + capacity - _size + i) % capacity];
    }
    return _size;
  }

private:
  static constexpr uint8_t capacity = 15;
  uint32_t _baud = 0;
  uint32_t _start = 0;
  uint32_t _highSince = 0;
  bool _synchronized = false;
  bool _receiving = false;
  bool _high = true;
  uint8_t _bit = 0;
  uint8_t _byte = 0;
  uint8_t _size = 0;
  uint8_t _next = 0;
  uint8_t _bytes[capacity] = {};
};

} // namespace probe
#endif
