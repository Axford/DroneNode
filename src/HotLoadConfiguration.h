#ifndef HOT_LOAD_CONFIGURATION_H
#define HOT_LOAD_CONFIGURATION_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace configuration {

enum class HotLoadResult : uint8_t {
  Idle, Pending, Applied, InvalidFilename, ReadFailed, InvalidConfiguration,
  RestartRequired, ResourceConflict, StartFailed, RecoveryFailed, OutOfMemory
};

inline bool hotLoadPath(const char* name, size_t size, char (&path)[18]) {
  if (size < 5 || size > 16 || memcmp(name + size - 4, ".ini", 4)) return false;
  for (size_t i = 0; i < size - 4; ++i) {
    const char c = name[i];
    if (!(c >= 'a' && c <= 'z') && !(c >= 'A' && c <= 'Z') &&
        !(c >= '0' && c <= '9') && c != '_' && c != '-') return false;
  }
  path[0] = '/';
  memcpy(path + 1, name, size);
  path[size + 1] = 0;
  return true;
}

}
#endif
