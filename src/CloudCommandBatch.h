#ifndef CLOUD_COMMAND_BATCH_H
#define CLOUD_COMMAND_BATCH_H
#include "TelemetryReport.h"

namespace telemetry {
struct CommandBatch {
  Packet packets[BatchLimit];
  size_t count = 0;

  // The response is small and its only string value is canonical ASCII base64.
  bool parse(const char* response, uint8_t node, size_t limit) {
    count = 0;
    const char* encoded = nullptr;
    size_t length = 0, expected = 0;
    if (!take(response, '{')) return false;
    bool success = false, data = false;
    for (unsigned i = 0; i < 2; ++i) {
      if (i && !take(response, ',')) return false;
      if (literal(response, "\"success\"")) {
        if (success || !take(response, ':') || !literal(response, "true")) return false;
        success = true;
      } else if (literal(response, "\"data\"")) {
        if (data || !take(response, ':') || !readData(response, encoded, length, expected)) return false;
        data = true;
      } else return false;
    }
    if (!take(response, '}') || *space(response) || !success || !data || expected > limit || expected > BatchLimit) return false;
    uint8_t raw[BatchLimit * 21];
    size_t size = 0;
    if (!decode(encoded, length, raw, size)) return false;
    size_t offset = 0;
    for (size_t i = 0; i < expected; ++i) {
      if (size - offset < 6) return false;
      const uint8_t* p = raw + offset;
      const size_t payload = (p[4] & 15) + 1;
      const unsigned type = (p[4] >> 4) & 7, priority = p[3] >> 6;
      if (payload + 5 > size - offset || p[0] != node || p[1] != node ||
          p[2] == 0 || p[2] == 255 || !(p[3] & 63) || (p[4] & 128)) return false;
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
      packets[i].size = payload + 5;
      memcpy(packets[i].bytes, p, packets[i].size);
      offset += packets[i].size;
    }
    if (offset != size) return false;
    count = expected;
    return true;
  }
private:
  static const char* space(const char*& p) {
    while (*p == ' ' || *p == '\r' || *p == '\n' || *p == '\t') ++p;
    return p;
  }
  static bool take(const char*& p, char c) {
    if (*space(p) != c) return false;
    ++p;
    return true;
  }
  static bool literal(const char*& p, const char* token) {
    space(p);
    const size_t n = strlen(token);
    if (strncmp(p, token, n)) return false;
    p += n;
    return true;
  }
  static bool readData(const char*& p, const char*& encoded, size_t& length, size_t& expected) {
    if (!take(p, '{')) return false;
    bool seenCount = false;
    for (unsigned i = 0; i < 2; ++i) {
      if (i && !take(p, ',')) return false;
      if (literal(p, "\"count\"")) {
        if (seenCount || !take(p, ':') || *space(p) < '0' || *p > '9') return false;
        seenCount = true;
        expected = *p++ - '0';
        if (*p >= '0' && *p <= '9') {
          if (!expected) return false;
          expected = expected * 10 + (*p++ - '0');
        }
      } else if (literal(p, "\"packets\"")) {
        if (encoded || !take(p, ':') || !take(p, '"')) return false;
        encoded = p;
        while (*p && *p != '"' && size_t(p - encoded) <= 896) ++p;
        length = p - encoded;
        if (length > 896 || *p != '"') return false;
        ++p;
      } else return false;
    }
    return seenCount && encoded && take(p, '}');
  }
  static int digit(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
  }
  static bool decode(const char* input, size_t length, uint8_t* output, size_t& size) {
    if (length % 4) return false;
    for (size_t i = 0; i < length; i += 4) {
      const int a = digit(input[i]), b = digit(input[i + 1]);
      const bool pad2 = input[i + 2] == '=', pad1 = input[i + 3] == '=';
      const int c = pad2 ? 0 : digit(input[i + 2]), d = pad1 ? 0 : digit(input[i + 3]);
      if (a < 0 || b < 0 || c < 0 || d < 0 ||
          ((pad1 || pad2) && i + 4 != length) || (pad2 && (!pad1 || (b & 15))) ||
          (pad1 && !pad2 && (c & 3))) return false;
      output[size++] = a << 2 | b >> 4;
      if (!pad2) output[size++] = b << 4 | c >> 2;
      if (!pad1) output[size++] = c << 6 | d;
    }
    return true;
  }
};

constexpr size_t ResponseLimit = 64;
class CommandResponses {
public:
  struct Window {
    uint8_t node = 0, channel = 0, param = 0, facet = 0;
    uint32_t deadline = 0;
  };
  void clear() {
    for (auto& w : windows) w = Window();
    queued = 0;
  }
  size_t capacity(uint32_t now) {
    expire(now);
    return ResponseLimit - queued - active();
  }
  Window* select(const Packet& query, uint32_t now) {
    expire(now);
    const uint8_t node = query.bytes[1], channel = query.bytes[2], param = query.bytes[3] & 63, facet = query.bytes[4] >> 4 == 6;
    for (auto& w : windows) if (w.node == node && w.channel == channel && w.param == param && w.facet == facet) return &w;
    if (queued + active() == ResponseLimit) return nullptr;
    for (auto& w : windows) if (!w.channel) return &w;
    return nullptr;
  }
  bool capture(const uint8_t* p, size_t size, uint32_t now) {
    if (!publication(p, size)) return false;
    expire(now);
    const uint8_t facet = ((p[4] >> 4) & 7) == 5;
    for (auto& w : windows) {
      if (w.node != p[1] || w.channel != p[2] || w.param != (p[3] & 63) || w.facet != facet) continue;
      if (queued >= ResponseLimit) return false;
      auto& packet = pending[queued++];
      packet.size = size;
      memcpy(packet.bytes, p, size);
      w = Window();
      return true;
    }
    return false;
  }
  bool pendingFacet(const Packet& packet) const {
    for (size_t i = 0; i < queued; ++i) if (sameFacet(pending[i], packet)) return true;
    return false;
  }
  bool take(Cycle& cycle) {
    cycle.count = 0;
    while (cycle.count < queued && cycle.count < BatchLimit) {
      const auto& packet = pending[cycle.count];
      bool duplicate = false;
      for (size_t i = 0; i < cycle.count; ++i) {
        if (sameFacet(cycle.entries[i].packet, packet)) { duplicate = true; break; }
      }
      if (duplicate) break;
      cycle.entries[cycle.count++].packet = packet;
    }
    if (!cycle.count) return false;
    queued -= cycle.count;
    memmove(pending, pending + cycle.count, queued * sizeof(Packet));
    return true;
  }
private:
  static bool sameFacet(const Packet& first, const Packet& second) {
    const auto* a = first.bytes;
    const auto* b = second.bytes;
    return a[1] == b[1] && a[2] == b[2] && (a[3] & 63) == (b[3] & 63) &&
        (((a[4] >> 4) & 7) == 5) == (((b[4] >> 4) & 7) == 5);
  }
  Window windows[ResponseLimit];
  Packet pending[ResponseLimit];
  size_t queued = 0;
  void expire(uint32_t now) {
    for (auto& w : windows) if (w.channel && int32_t(now - w.deadline) >= 0) w = Window();
  }
  size_t active() const {
    size_t count = 0;
    for (const auto& w : windows) if (w.channel) ++count;
    return count;
  }
};
}
#endif
