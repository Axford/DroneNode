#pragma once
#include <stdint.h>

namespace recovery {
enum class Phase : uint8_t {
  Storage, Hardware, WiFi, Configuration, ModuleConstruction, ModuleSetup,
  Web, Probation, Ready
};

struct Breadcrumb {
  Phase phase = Phase::Storage;
  uint8_t module = 0;
  char type[24] = {};
};

struct BootRecord {
  Breadcrumb current;
  Breadcrumb failure;
  uint8_t failures = 0;
  uint8_t failureReason = 0;
  bool recovering = false;
  bool retry = false;

  void begin(bool interrupted, bool softwareRestart, uint8_t reason) {
    if (interrupted && (recovering || current.phase != Phase::Ready)) {
      failure = current;
      failureReason = reason;
      if (recovering) failures = 2;
      else if (failures < 2) ++failures;
    }
    if (recovering && softwareRestart && !retry) failures = 2;
    recovering = failures && !(softwareRestart && retry);
    retry = false;
    current = Breadcrumb{};
  }

  void ready() {
    current.phase = Phase::Ready;
    if (!recovering) failures = 0;
  }

  bool mayRetry() const { return recovering && failures == 1; }

  void requestRetry(bool explicitRequest) {
    if (!recovering) return;
    if (explicitRequest) failures = 1;
    retry = true;
  }
};
}
