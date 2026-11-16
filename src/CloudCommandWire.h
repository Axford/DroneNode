#ifndef CLOUD_COMMAND_WIRE_H
#define CLOUD_COMMAND_WIRE_H
#include <stddef.h>
#include <stdint.h>

namespace telemetry {
inline bool validCommandPacket(const uint8_t* p, size_t size) {
  if (size < 6 || size > 21) return false;
  const size_t payload = (p[4] & 15) + 1;
  const unsigned type = (p[4] >> 4) & 7, priority = p[3] >> 6;
  if (size != payload + 5 || !p[2] || p[2] == 255 || !(p[3] & 63) || (p[4] & 128)) return false;
  if (type <= 4) {
    if (priority != 3 || (type >= 1 && type <= 3 && payload % 4)) return false;
    if (type == 3) for (size_t j = 0; j < payload; j += 4) {
      const uint8_t* f = p + 5 + j;
      if ((f[3] & 127) == 127 && (f[2] & 128)) return false;
      if (f[3] == 128 && !(f[0] | f[1] | f[2])) return false;
    }
    if (type == 4) {
      bool padding = false;
      for (size_t j = 0; j < payload; ++j) {
        if (!p[5 + j]) padding = true;
        else if (padding) return false;
      }
    }
  } else if (type == 5 || priority != 0 || payload != 1 || p[5] != 0) return false;
  return true;
}
}
#endif
