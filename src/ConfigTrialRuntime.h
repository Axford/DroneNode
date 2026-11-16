#pragma once

#include "ConfigTrial.h"

namespace configTrial {
struct Report {
  const char* phase = "idle";
  bool hasInstalled = false;
  bool hasStaged = false;
  Digest installed;
  Digest staged;
};
Report report();
Boot select();
bool running();
bool writable(const char* path);
bool stage(const uint8_t* bytes, size_t size, const Digest& installed, const Digest& candidate);
bool arm(const Digest& installed, const Digest& candidate);
bool confirm(const Digest& candidate);
bool completedPass(uint32_t now);
void checkDeadline(uint32_t now);
void retryInstalled();
void fail();

#ifdef CONFIG_TRIAL_BENCH
void benchCommand(const char* command);
#endif
}
