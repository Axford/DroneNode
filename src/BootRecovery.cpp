#include "BootRecovery.h"
#include "ConfigurationLine.h"
#include "QuarantineConfiguration.h"
#include <mbedtls/sha256.h>
#include <memory>
#include <new>

#include <Arduino.h>
#include <atomic>
#include <Preferences.h>
#include <esp_attr.h>
#include <esp_system.h>

namespace {
constexpr uint32_t RecordVersion = 0x44524e02;
constexpr uint32_t HealthyTimeMs = 10000;
constexpr uint32_t RetryTimeMs = 60000;
struct RetainedBoot {
  uint32_t version;
  recovery::BootRecord record;
};
static_assert(__is_trivially_copyable(RetainedBoot), "RTC record must be directly copyable");
RTC_NOINIT_ATTR uint8_t retainedBytes[sizeof(RetainedBoot)];
RetainedBoot retained;
RecoveryProfile saved, candidate;
recovery::Breadcrumb failureAtBoot;
uint8_t reasonAtBoot;
uint32_t firstPass;
bool observedPass = false;
bool healthy = false;
std::atomic<bool> automaticRetryDeferred{false};

void retain() {
  memcpy(retainedBytes, &retained, sizeof retained);
}

bool number(const char* text, unsigned limit, unsigned& value) {
  if (!*text) return false;
  value = 0;
  for (; *text; ++text) {
    if (*text < '0' || *text > '9') return false;
    value = value * 10 + (*text - '0');
    if (value > limit) return false;
  }
  return true;
}

bool address(const char* text, uint8_t (&ip)[4]) {
  for (unsigned i = 0; i < 4; ++i) {
    unsigned value = 0, digits = 0;
    while (*text >= '0' && *text <= '9') {
      value = value * 10 + (*text++ - '0');
      if (++digits > 3 || value > 255) return false;
    }
    if (!digits || (i == 3 ? *text != 0 : *text++ != ',')) return false;
    ip[i] = value;
  }
  return true;
}

void saveCandidate() {
  if (!candidate.valid() || memcmp(&saved, &candidate, sizeof saved) == 0) return;
  Preferences prefs;
  if (prefs.begin("boot-recovery", false) &&
      prefs.putBytes("wifi-v1", &candidate, sizeof candidate) == sizeof candidate) {
    saved = candidate;
  }
}
}

namespace bootRecovery {
void begin() {
  const auto reason = esp_reset_reason();
  memcpy(&retained, retainedBytes, sizeof retained);
  if (retained.version != RecordVersion || reason == ESP_RST_POWERON || reason == ESP_RST_BROWNOUT) {
    retained = RetainedBoot{};
    retained.version = RecordVersion;
  }
  const bool interrupted = reason == ESP_RST_TASK_WDT || reason == ESP_RST_INT_WDT ||
                           reason == ESP_RST_WDT || reason == ESP_RST_PANIC;
  retained.record.begin(interrupted, reason == ESP_RST_SW, reason, reason == ESP_RST_TASK_WDT);
  failureAtBoot = retained.record.failure;
  reasonAtBoot = retained.record.failureReason;
  retain();

  Preferences prefs;
  if (prefs.begin("boot-recovery", true) &&
      prefs.getBytesLength("wifi-v1") == sizeof saved) {
    prefs.getBytes("wifi-v1", &saved, sizeof saved);
  }
}

bool active() { return retained.record.recovering; }
uint8_t quarantinedModule() { return retained.record.quarantined; }
uint8_t optionalStatus() { return retained.record.config.optionalStatus; }

void inspectConfig(fs::FS& fs) {
  if (active() && !retained.record.quarantinePending) return;
  recovery::ConfigIdentity config;
  File file = fs.open("/config.ini");
  if (file && !file.isDirectory()) {
    const size_t size = file.size();
    if (size && size <= 8192) {
      std::unique_ptr<char[]> text(new (std::nothrow) char[size + 1]);
      if (text && file.read(reinterpret_cast<uint8_t*>(text.get()), size) == size &&
          !memchr(text.get(), 0, size)) {
        text[size] = 0;
        config.optionalStatus = recovery::approvedStatus(text.get());
        if (config.optionalStatus &&
            mbedtls_sha256_ret(reinterpret_cast<const uint8_t*>(text.get()), size, config.hash, 0))
          config.optionalStatus = 0;
      }
    }
  }
  retained.record.inspectConfig(config);
  retain();
}
const RecoveryProfile& profile() { return saved; }
const recovery::Breadcrumb& failure() { return failureAtBoot; }
uint8_t failureReason() { return reasonAtBoot; }

void enter(recovery::Phase phase, uint8_t module, const char* type) {
  auto& point = retained.record.current;
  point.phase = phase;
  point.module = module;
  strlcpy(point.type, type, sizeof point.type);
  retain();
}

void requestRestart() {
  retained.record.requestRetry(true);
  retain();
}

void forceRecovery() {
  retained.record.recovering = true;
  retained.record.failures = 2;
  retained.record.quarantined = 0;
  retain();
}

void deferAutomaticRetry() {
  automaticRetryDeferred.store(true);
}

void readCandidate(fs::FS& fs) {
  File file = fs.open("/config.ini");
  if (!file) return;

  RecoveryProfile parsed;
  bool inSection = false, udp = false, foundUdp = false;
  char raw[2049];
  while (file.available()) {
    const size_t length = file.readBytesUntil('\n', raw, sizeof raw - 1);
    if (length == sizeof raw - 1) return;
    raw[length] = 0;
    configuration::Line line;
    if (!line.parse(raw, true)) {
      if (!inSection || udp) return;
      continue;
    }
    if (line.empty) continue;
    unsigned value;
    if (line.section) {
      inSection = true;
      udp = !foundUdp && !strcmp(line.name, "UDPTelemetry");
      if (udp) {
        if (!number(line.value, 254, value) || !value) return;
        parsed.channel = value == 1 ? 2 : value;
        foundUdp = true;
      }
    } else if (!inSection && !strcmp(line.name, "node")) {
      if (parsed.node || !number(line.value, 254, value) || !value) return;
      parsed.node = value;
    } else if (udp && !strcmp(line.name, "port")) {
      if (!number(line.value, 65535, value) || !value) return;
      parsed.port = value;
    } else if (udp && !strcmp(line.name, "broadcast")) {
      if (!address(line.value, parsed.broadcast)) return;
    } else if (udp && !strcmp(line.name, "server")) {
      if (!address(line.value, parsed.server)) return;
    }
  }
  if (!parsed.valid()) return;
  candidate = parsed;
  if (!saved.valid()) saveCandidate();
}

void completedPass(uint32_t now) {
  if (!observedPass) {
    observedPass = true;
    firstPass = now;
  }
  const uint32_t elapsed = now - firstPass;
  if (!healthy && elapsed >= HealthyTimeMs) {
    if (!active() && !quarantinedModule()) saveCandidate();
    retained.record.ready();
    retain();
    healthy = true;
  }
  if (elapsed >= RetryTimeMs && retained.record.mayRetry() && !automaticRetryDeferred.load()) {
    retained.record.requestRetry(false);
    retain();
    ESP.restart();
  }
}
}
