#pragma once
#include <stdint.h>
#include <string.h>

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

struct ConfigIdentity {
  uint8_t hash[32] = {};
  uint8_t optionalStatus = 0;

  bool matches(const ConfigIdentity& other) const {
    return optionalStatus && optionalStatus == other.optionalStatus &&
           memcmp(hash, other.hash, sizeof hash) == 0;
  }
};

struct BootRecord {
  Breadcrumb current;
  Breadcrumb failure;
  ConfigIdentity config;
  ConfigIdentity failedConfig;
  uint8_t quarantined = 0;
  bool quarantinePending = false;
  uint8_t failures = 0;
  uint8_t failureReason = 0;
  bool recovering = false;
  bool retry = false;

  void begin(bool interrupted, bool softwareRestart, uint8_t reason, bool taskWatchdog = false) {
    quarantinePending = false;
    if (quarantined) {
      if (softwareRestart && retry) {
        failures = 0;
      } else {
        failures = 2;
        if (interrupted) {
          failure = current;
          failureReason = reason;
        }
      }
    } else if (interrupted && (recovering || current.phase != Phase::Ready)) {
      quarantinePending = !recovering && failures == 1 && taskWatchdog && failureReason == reason &&
                          current.phase == Phase::ModuleSetup && failure.phase == Phase::ModuleSetup &&
                          config.matches(failedConfig) && current.module == config.optionalStatus &&
                          failure.module == current.module &&
                          strcmp(current.type, "Status") == 0 && strcmp(failure.type, current.type) == 0;
      failure = current;
      failureReason = reason;
      failedConfig = config;
      if (recovering) failures = 2;
      else if (failures < 2) ++failures;
    }
    if (recovering && softwareRestart && !retry) failures = 2;
    recovering = failures && !(softwareRestart && retry);
    quarantined = 0;
    retry = false;
    current = Breadcrumb{};
  }

  void inspectConfig(const ConfigIdentity& inspected) {
    if (quarantinePending && config.matches(inspected)) {
      quarantined = inspected.optionalStatus;
      recovering = false;
    }
    quarantinePending = false;
    config = inspected;
  }

  void ready() {
    current.phase = Phase::Ready;
    if (!recovering && !quarantined) failures = 0;
  }

  bool mayRetry() const { return recovering && failures == 1; }

  void requestRetry(bool explicitRequest) {
    if (!recovering && !quarantined) return;
    if (recovering && explicitRequest) {
      failures = 1;
      failedConfig = ConfigIdentity{};
    }
    retry = true;
  }
};
}
