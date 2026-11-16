#include "CloudRecovery.h"

#include "BootRecovery.h"
#include "CloudTelemetryHttp.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <atomic>
#include <memory>
#include <new>

namespace {
constexpr uint8_t Version = 1;
constexpr uint8_t StatusParam = 23;
constexpr uint8_t RetryParam = 24;
constexpr uint32_t PollMs = 5000;
constexpr uint32_t StatusMs = 10000;
enum class Mode : uint8_t { Starting, Normal, Quarantined, CommsOnly };

Mode currentMode() {
  if (bootRecovery::active()) return Mode::CommsOnly;
  if (bootRecovery::quarantinedModule()) return Mode::Quarantined;
  return bootRecovery::healthy() ? Mode::Normal : Mode::Starting;
}

struct Worker : telemetry::HttpIo {
  telemetry::HttpSettings settings;
  char certificate[2049];
  uint8_t node, reason, phase, fault, quarantine;
  std::atomic<Mode> mode{Mode::Starting};
  std::atomic<bool> retry{false};
  WiFiClientSecure client;

  uint32_t now() override { return millis(); }
  bool running() override { return WiFi.status() == WL_CONNECTED; }
  bool connect(const char* host, uint16_t port) override {
    return running() && (client.connected() || client.connect(host, port, 5000));
  }
  size_t write(const uint8_t* bytes, size_t size) override { return running() ? client.write(bytes, size) : 0; }
  int read() override { return running() && client.available() ? client.read() : -1; }
  bool connected() override { return client.connected(); }
  void wait() override { delay(5); }
  void close() override { client.stop(); }

  void run() {
    client.setCACert(certificate);
    client.setHandshakeTimeout(5);
    client.setTimeout(5);
    telemetry::PackedHttp http(*this);
    telemetry::Cycle report;
    auto& packet = report.entries[0].packet;
    report.count = 1;
    packet.size = 19;
    const uint8_t status[] = {node, node, 1, StatusParam, 13, Version, 0, reason, phase, fault, quarantine};
    memcpy(packet.bytes, status, sizeof status);
    auto* bootId = packet.bytes + sizeof status;
    Mode lastMode = Mode::Starting;
    uint32_t lastReport = 0;
    bool identified = false, reportDue = true, retryRequested = false;
    for (;;) {
      if (!running()) {
        close();
        reportDue = true;
      } else {
        if (!identified) {
          // Wi-Fi is running, so the ESP32 RNG has its RF entropy source.
          esp_fill_random(bootId, 8);
          identified = true;
        }
        const Mode current = mode.load();
        if (reportDue || current != lastMode || uint32_t(millis() - lastReport) >= StatusMs) {
          packet.bytes[6] = static_cast<uint8_t>(current);
          const auto result = http.sendBatch(settings, report, 0, 1, node);
          close();
          reportDue = !result.complete || result.admitted != 1;
          lastMode = current;
          lastReport = millis();
        }
        if (!retryRequested && (current == Mode::Quarantined || current == Mode::CommsOnly)) {
          telemetry::CommandBatch batch;
          const auto result = http.poll(settings, node, 1, batch, telemetry::PollPeers(), bootId);
          if (result.complete && batch.count == 1) {
            const auto& action = batch.packets[0];
            const uint8_t header[] = {node, node, 1, uint8_t(0xc0 | RetryParam), 8, Version};
            if (action.size == 14 && !memcmp(action.bytes, header, sizeof header) &&
                !memcmp(action.bytes + sizeof header, bootId, 8)) {
              retryRequested = true;
              retry.store(true);
            }
          }
        }
      }
      delay(PollMs);
    }
  }
};

Worker* worker = nullptr;
}

namespace cloudRecovery {
void start(uint8_t node) {
  if (worker || node == 0 || node == 255) return;
  Preferences preferences;
  if (!preferences.begin("cloud-recovery", true)) return;
  const size_t size = preferences.getBytesLength("https-v1");
  if (!size || size > 4096) return;
  std::unique_ptr<char[]> json(new (std::nothrow) char[size]);
  if (!json || preferences.getBytes("https-v1", json.get(), size) != size) return;
  preferences.end();
  DynamicJsonDocument profile(4096);
  if (deserializeJson(profile, json.get(), size)) return;
  const char* certificate = profile["ca"];
  if (!telemetry::printable(profile["endpoint"], 256) || !certificate ||
      strncmp(certificate, "-----BEGIN CERTIFICATE-----", 27) || strlen(certificate) > 2048) return;
  std::unique_ptr<Worker> candidate(new (std::nothrow) Worker);
  if (!candidate || !candidate->settings.parseEndpoint(profile["endpoint"]) ||
      !candidate->settings.setDomain(profile["domain"]) || !candidate->settings.setApiKey(profile["apiKey"])) return;
  strlcpy(candidate->certificate, certificate, sizeof candidate->certificate);
  candidate->node = node;
  candidate->reason = bootRecovery::failureReason();
  candidate->phase = static_cast<uint8_t>(bootRecovery::failure().phase);
  candidate->fault = bootRecovery::failure().module;
  candidate->quarantine = bootRecovery::quarantinedModule();
  candidate->mode.store(currentMode());

  if (xTaskCreate([](void* context) { static_cast<Worker*>(context)->run(); },
      "cloud-recovery", 12288, candidate.get(), 1, nullptr) == pdPASS) {
    worker = candidate.release();
  }
}

void service() {
  if (!worker) return;
  const Mode mode = currentMode();
  worker->mode.store(mode);
  if (worker->retry.exchange(false) && (mode == Mode::Quarantined || mode == Mode::CommsOnly)) {
    bootRecovery::requestRestart();
    ESP.restart();
  }
}
}
