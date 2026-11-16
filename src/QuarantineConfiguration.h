#pragma once

#include "ConfigurationLine.h"
#include <stdint.h>

namespace recovery {
inline uint8_t channelNumber(const char* value) {
  unsigned id = 0;
  for (; *value; ++value) {
    if (*value < '0' || *value > '9') return 0;
    id = id * 10 + (*value - '0');
    if (id > 254) return 0;
  }
  return id;
}

template<class Visitor>
bool visitConfiguration(char* text, Visitor visit) {
  for (char* raw = text; raw;) {
    char* next = strchr(raw, '\n');
    if (next) *next = 0;
    configuration::Line line;
    const bool valid = strlen(raw) <= 2048 && line.parse(raw, true) &&
                       (line.empty || visit(line));
    if (next) *next = '\n';
    if (!valid) return false;
    raw = next ? next + 1 : nullptr;
  }
  return true;
}

inline uint8_t approvedStatus(char* text) {
  bool inSection = false, seen[255] = {};
  bool selected = false, found = false;
  uint8_t optional = 0;
  char name[17] = {};
  const bool valid = visitConfiguration(text, [&](const configuration::Line& line) {
    if (line.section) {
      inSection = true;
      const auto id = channelNumber(line.value);
      if (!id || seen[id]) return false;
      seen[id] = true;
      selected = optional && id == optional;
      if (selected) {
        if (strcmp(line.name, "Status")) return false;
        found = true;
      }
    } else if (!strcmp(line.name, "optionalStatus")) {
      if (inSection || optional) return false;
      optional = channelNumber(line.value);
      if (!optional) return false;
    } else if (selected && (!strcmp(line.name, "publish") || line.name[0] == '$')) {
      return false;
    } else if (selected && !strcmp(line.name, "name")) {
      if (strlen(line.value) >= sizeof name) return false;
      strcpy(name, line.value);
    }
    return true;
  });
  if (!valid || !found) return 0;

  const bool unreferenced = visitConfiguration(text, [&](const configuration::Line& line) {
    if (line.name[0] != '$') return true;
    const char* separator = strchr(line.value, '>');
    if (!separator) return false;
    const char* channel = separator + 1;
    const char* end = strchr(channel, '.');
    if (!end || end == channel || end - channel > 19) return false;
    char address[20] = {};
    memcpy(address, channel, end - channel);
    return channelNumber(address) != optional && (!*name || strcmp(address, name));
  });
  return unreferenced ? optional : 0;
}
}
