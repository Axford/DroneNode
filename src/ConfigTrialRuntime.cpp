#include <Arduino.h>
#include "ConfigTrialRuntime.h"

#include "BootRecovery.h"
#include "ConfigTrialStorage.h"
#include "DroneConsole.h"
#include <LittleFS.h>
#include <atomic>
#include <esp_attr.h>
#include <esp_system.h>
#ifdef CONFIG_TRIAL_BENCH
#include <memory>
#include <new>
#endif

namespace configTrial {
namespace {
constexpr uint32_t Activate = 0x43545231;
constexpr uint32_t ProbationMs = 10000;
constexpr uint32_t DeadlineMs = 60000;
RTC_NOINIT_ATTR uint32_t activation;
LittleFsStore store(LittleFS);
Transaction<LittleFsStore> transaction(store);
std::atomic<bool> inTrial{false};
uint32_t startedAt;
uint32_t firstPassAt;
bool firstPass;
bool ready;
bool confirmed;

bool matchesPath(const char* path, const char* name) {
  return strcmp(path, name) == 0;
}
}

Boot select() {
  const bool activate = esp_reset_reason() == ESP_RST_SW && activation == Activate;
  activation = 0;
  const auto decision = transaction.boot(activate);
  if (decision == Boot::Candidate) {
    startedAt = millis();
    inTrial.store(true);
  }
  return decision;
}

bool running() { return inTrial.load(); }

bool writable(const char* path) {
  // Local file endpoints permit root and nested paths. Reject ambiguous path
  // spellings so reserved names cannot be reached through aliases.
  if (!path || path[0] != '/') return false;
  for (const char* part = path + 1; ; ) {
    const char* end = strchr(part, '/');
    const size_t size = end ? static_cast<size_t>(end - part) : strlen(part);
    if (!size || (part[0] == '.' && (size == 1 || (size == 2 && part[1] == '.')))) return false;
    if (!end) break;
    part = end + 1;
  }
  return strncmp(path, "/.repair", 8) != 0 && !matchesPath(path, CandidatePath) &&
         !(running() && matchesPath(path, InstalledPath));
}

bool stage(const uint8_t* bytes, size_t size, const Digest& installed, const Digest& candidate) {
  return bootRecovery::active() && bootRecovery::profile().valid() &&
         transaction.stage(bytes, size, installed, candidate);
}

bool arm(const Digest& installed, const Digest& candidate) {
  if (!bootRecovery::active() || !bootRecovery::profile().valid() ||
      !transaction.arm(installed, candidate)) return false;
  activation = Activate;
  return true;
}

bool confirm(const Digest& candidate) {
  Digest current;
  if (!running() || !ready || millis() - startedAt >= DeadlineMs ||
      !store.digest(CandidatePath, current) || !(current == candidate)) return false;
  confirmed = transaction.commit(candidate);
  if (!confirmed) fail();
  return confirmed;
}

void checkDeadline(uint32_t now) {
  if (running() && now - startedAt >= DeadlineMs && !confirmed) fail();
}

void retryInstalled() {
  Digest installed;
  if (store.digest(InstalledPath, installed)) transaction.retryInstalled(installed);
}

bool completedPass(uint32_t now) {
  checkDeadline(now);
  if (!firstPass) {
    firstPass = true;
    firstPassAt = now;
  }
  if (!ready && now - firstPassAt >= ProbationMs) {
    ready = true;
    DroneConsole.println("config trial ready");
  }
  if (!confirmed) return false;
  inTrial.store(false);
  DroneConsole.println("config trial committed");
  return true;
}

void fail() {
  transaction.reject();
  bootRecovery::forceRecovery();
  ESP.restart();
}

#ifdef CONFIG_TRIAL_BENCH
void benchCommand(const char* command) {
  if (strcmp(command, "bench recover") == 0) {
    DroneConsole.println("bench recover ok");
    bootRecovery::forceRecovery();
    delay(50);
    ESP.restart();
  }
  if (strcmp(command, "bench stage") == 0) {
    constexpr const char* input = "/bench.ini";
    Digest installed, candidate;
    auto file = LittleFS.open(input, FILE_READ);
    const size_t size = file ? file.size() : 0;
    std::unique_ptr<uint8_t[]> bytes(size && size <= MaxConfigBytes ? new (std::nothrow) uint8_t[size] : nullptr);
    const bool read = bytes && file.read(bytes.get(), size) == size;
    file.close();
    const bool staged = read && store.digest(InstalledPath, installed) && store.digest(input, candidate) &&
                        stage(bytes.get(), size, installed, candidate);
    if (staged) LittleFS.remove(input);
    DroneConsole.println(staged ? "bench stage ok" : "bench stage error");
  }
  if (strcmp(command, "bench trial") == 0) {
    Digest installed, candidate;
    if (store.digest(InstalledPath, installed) && store.digest(CandidatePath, candidate) &&
        arm(installed, candidate)) {
      DroneConsole.println("bench trial ok");
      delay(50);
      ESP.restart();
    }
    DroneConsole.println("bench trial error");
  }
  if (strcmp(command, "bench ack") == 0) {
    Digest candidate;
    const bool accepted = store.digest(CandidatePath, candidate) && confirm(candidate);
    DroneConsole.println(accepted ? "bench ack ok" : "bench ack error");
  }
  if (strcmp(command, "bench cleanup") == 0 && !running()) {
    LittleFS.remove(CandidatePath);
    LittleFS.remove("/.repair/previous.ini");
    LittleFS.remove("/.repair/pending-backup.ini");
    LittleFS.remove("/.repair/state");
    LittleFS.remove("/.repair/write");
    LittleFS.rmdir("/.repair");
    DroneConsole.println("bench cleanup ok");
  }
}
#endif
}
