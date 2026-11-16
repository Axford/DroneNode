#include "CloudRecovery.h"

#include "BootRecovery.h"
#include "CloudTelemetryHttp.h"
#include "ConfigTrialRuntime.h"
#include "DroneConsole.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <atomic>
#include <mbedtls/base64.h>
#include <mbedtls/ecdsa.h>
#include <mbedtls/sha256.h>
#include <memory>
#include <new>

namespace {
constexpr char PreferenceSpace[] = "node-identity";
constexpr uint32_t PollMs = 5000;
constexpr uint32_t StatusMs = 10000;
constexpr uint8_t Version = 1;
enum class Mode : uint8_t { Starting, Normal, Quarantined, CommsOnly };

struct DeviceKey {
  uint8_t secret[32];
  uint8_t point[65];
};

int entropy(void*, unsigned char* bytes, size_t size) {
  if (WiFi.status() != WL_CONNECTED) return -1;
  esp_fill_random(bytes, size);
  return 0;
}

bool makeKey(DeviceKey& key) {
  mbedtls_ecp_keypair pair;
  mbedtls_ecp_keypair_init(&pair);
  size_t size = 0;
  const bool ok = !mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, &pair, entropy, nullptr) &&
    !mbedtls_mpi_write_binary(&pair.d, key.secret, sizeof key.secret) &&
    !mbedtls_ecp_point_write_binary(&pair.grp, &pair.Q, MBEDTLS_ECP_PF_UNCOMPRESSED, &size, key.point, sizeof key.point) &&
    size == sizeof key.point;
  mbedtls_ecp_keypair_free(&pair);
  return ok;
}

void encodeHex(const uint8_t* bytes, size_t size, char* out) {
  constexpr char digits[] = "0123456789abcdef";
  while (size--) {
    *out++ = digits[*bytes >> 4];
    *out++ = digits[*bytes++ & 15];
  }
  *out = 0;
}

bool lowerHex(const char* value, size_t size) {
  return value && strlen(value) == size && strspn(value, "0123456789abcdef") == size;
}

bool signNonce(const DeviceKey& key, const char* nonce, char* signature) {
  uint8_t digest[32], bytes[64];
  mbedtls_ecp_group group;
  mbedtls_ecp_group_init(&group);
  mbedtls_mpi d, r, s;
  mbedtls_mpi_init(&d); mbedtls_mpi_init(&r); mbedtls_mpi_init(&s);
  const bool ok =
    !mbedtls_sha256_ret(reinterpret_cast<const uint8_t*>(nonce), 64, digest, 0) &&
    !mbedtls_ecp_group_load(&group, MBEDTLS_ECP_DP_SECP256R1) &&
    !mbedtls_mpi_read_binary(&d, key.secret, sizeof key.secret) &&
    !mbedtls_ecdsa_sign(&group, &r, &s, &d, digest, sizeof digest, entropy, nullptr) &&
    !mbedtls_mpi_write_binary(&r, bytes, 32) && !mbedtls_mpi_write_binary(&s, bytes + 32, 32);
  mbedtls_mpi_free(&d); mbedtls_mpi_free(&r); mbedtls_mpi_free(&s);
  mbedtls_ecp_group_free(&group);
  if (ok) encodeHex(bytes, sizeof bytes, signature);
  return ok;
}

Mode currentMode() {
  if (bootRecovery::active()) return Mode::CommsOnly;
  if (bootRecovery::quarantinedModule()) return Mode::Quarantined;
  return bootRecovery::healthy() ? Mode::Normal : Mode::Starting;
}

struct NodeRecoveryProfile {
  telemetry::HttpSettings http;
  char certificate[2049];
  char enrollment[33];
  uint8_t node;

  bool parse(char* input) {
    char* fields[5];
    for (size_t i = 0; i < 4; ++i) {
      fields[i] = input;
      char* split = strchr(input, '|');
      if (!split) return false;
      *split = 0;
      input = split + 1;
    }
    fields[4] = input;
    unsigned address;
    if (!telemetry::decimal(fields[2], 254, address) || !lowerHex(fields[3], 32) ||
        !http.parseEndpoint(fields[0]) || !http.setDomain(fields[1]) ||
        strlen(fields[1]) != strspn(fields[1], "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-") ||
        strncmp(fields[4], "-----BEGIN CERTIFICATE-----", 27) || strlen(fields[4]) >= sizeof certificate) return false;
    node = address;
    strcpy(enrollment, fields[3]);
    strcpy(certificate, fields[4]);
    for (char* ch = certificate; *ch; ++ch) if (*ch == '~') *ch = '\n';
    return true;
  }
};

struct RepairExchange {
  enum class Step : uint8_t { Idle, ReadFiles, Report, Action };
  std::atomic<Step> step{Step::Idle};
  configTrial::Report report;
  std::unique_ptr<char[]> response;
  bool disabled = false;
#ifdef CONFIG_TRIAL_BENCH
  std::atomic<bool> dropConfirmation{false};
#endif
  static constexpr size_t Header = 129;
  static constexpr size_t Capacity = Header + configTrial::MaxConfigBytes + 1;

  static bool decode(const char* text, configTrial::Digest& value) {
    for (size_t i = 0; i < sizeof value.bytes; ++i) {
      const auto digit = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
      };
      const int first = digit(text[2 * i]), second = digit(text[2 * i + 1]);
      if (first < 0 || second < 0) return false;
      value.bytes[i] = (first << 4) | second;
    }
    return true;
  }

  void poll(telemetry::PackedHttp& http, const NodeRecoveryProfile& profile, const char* boot, Mode mode, char* token) {
    if (disabled || !token[0]) return;
    if (!response) response.reset(new (std::nothrow) char[Capacity]);
    if (!response) return;
    if (step.load() == Step::Idle) {
      step.store(Step::ReadFiles);
      for (unsigned tick = 0; tick < 30 && step.load() == Step::ReadFiles; ++tick) delay(5);
    }
    if (step.load() != Step::Report) return;
    char installed[65] = {}, staged[65] = {}, request[455];
    if (report.hasInstalled) encodeHex(report.installed.bytes, 32, installed);
    if (report.hasStaged) encodeHex(report.staged.bytes, 32, staged);
    snprintf(request, sizeof request,
        "{\"domain\":\"%s\",\"id\":%u,\"boot\":\"%s\",\"mode\":%u,\"phase\":\"%s\",\"installed\":\"%s\",\"staged\":\"%s\"}",
        profile.http.domain, profile.node, boot, unsigned(mode), report.phase, installed, staged);
    int status;
    const bool ok = http.postJson(profile.http, "/node-auth/repair", request, token, response.get(), Capacity, status);
    if (status == 404) { disabled = true; response.reset(); }
    if (status == 401) token[0] = 0;
    if (ok && status == 200 && strlen(response.get()) >= Header) {
#ifdef CONFIG_TRIAL_BENCH
      if (response[0] == 'c' && dropConfirmation.load()) {
        DroneConsole.println("bench confirmation dropped");
        step.store(Step::Idle);
        return;
      }
#endif
      step.store(Step::Action);
    } else step.store(Step::Idle);
  }

  void service(Mode mode) {
    const Step current = step.load();
    if (current == Step::ReadFiles) {
      report = configTrial::report();
      step.store(Step::Report);
      return;
    }
    if (current != Step::Action) return;
    configTrial::Digest installed, candidate;
    const size_t size = strlen(response.get());
    if (decode(response.get() + 1, installed) && decode(response.get() + 65, candidate)) {
      const char operation = response[0];
      if (operation == 's' && mode == Mode::CommsOnly && size > Header)
        configTrial::stage(reinterpret_cast<const uint8_t*>(response.get() + Header), size - Header, installed, candidate);
      if (operation == 'a' && mode == Mode::CommsOnly && size == Header && configTrial::arm(installed, candidate))
        ESP.restart();
      if (operation == 'c' && size == Header) configTrial::confirm(candidate);
    }
    step.store(Step::Idle);
  }
};

struct Worker : telemetry::HttpIo {
  NodeRecoveryProfile profile;
  DeviceKey key;
  uint8_t reason, phase, fault, quarantine;
  char token[98] = {};
  uint32_t refreshAt = 0;
  std::atomic<Mode> mode{Mode::Starting};
  std::atomic<bool> retry{false};
  WiFiClientSecure client;
  telemetry::CommandBatch batch;
  RepairExchange repair;

  uint32_t now() override { return millis(); }
  bool running() override { return WiFi.status() == WL_CONNECTED; }
  bool connect(const char* host, uint16_t port) override { return running() && client.connect(host, port, 5000); }
  size_t write(const uint8_t* bytes, size_t size) override { return running() ? client.write(bytes, size) : 0; }
  int read() override { return running() && client.available() ? client.read() : -1; }
  bool connected() override { return client.connected(); }
  void wait() override { delay(5); }
  void close() override { client.stop(); }

  bool renew(telemetry::PackedHttp& http) {
    char input[512], response[384], nonce[65], signature[129];
    int status;
    snprintf(input, sizeof input, "{\"domain\":\"%s\",\"id\":%u,\"enrollmentId\":\"%s\"}", profile.http.domain, profile.node, profile.enrollment);
    if (!http.postJson(profile.http, "/node-auth/challenge", input, nullptr, response, sizeof response, status) || status != 200) return false;
    StaticJsonDocument<384> json;
    if (deserializeJson(json, response) || !json["success"].as<bool>()) return false;
    const char* value = json["data"]["nonce"];
    if (!lowerHex(value, 64)) return false;
    strcpy(nonce, value);
    if (!signNonce(key, nonce, signature)) return false;
    snprintf(input, sizeof input, "{\"domain\":\"%s\",\"id\":%u,\"enrollmentId\":\"%s\",\"nonce\":\"%s\",\"signature\":\"%s\"}",
      profile.http.domain, profile.node, profile.enrollment, nonce, signature);
    const uint32_t requestedAt = millis();
    if (!http.postJson(profile.http, "/node-auth/renew", input, nullptr, response, sizeof response, status) || status != 200) return false;
    if (deserializeJson(json, response) || !json["success"].as<bool>()) return false;
    const char* bearer = json["data"]["token"];
    const unsigned seconds = json["data"]["expiresIn"] | 0;
    if (!bearer || strlen(bearer) != 97 || strncmp(bearer, profile.enrollment, 32) ||
        bearer[32] != '.' || !lowerHex(bearer + 33, 64) || seconds < 15 || seconds > 3600) return false;
    strcpy(token, bearer);
    refreshAt = requestedAt + (seconds - 5) * 1000;
    return true;
  }

  void run() {
    client.setCACert(profile.certificate);
    client.setHandshakeTimeout(5);
    client.setTimeout(5);
    telemetry::PackedHttp http(*this);
    const uint8_t node = profile.node;
    uint8_t statusPacket[19] = {node, node, 1, 23, 13, Version, 0, reason, phase, fault, quarantine};
    char bootHex[17];
    auto* bootId = statusPacket + 11;
    Mode lastMode = Mode::Starting;
    uint32_t lastReport = 0;
    bool identified = false, reportDue = true, retryRequested = false;
    for (;;) {
      if (token[0] && int32_t(millis() - refreshAt) >= 0) token[0] = 0;
      if (!running()) {
        close();
        reportDue = true;
      } else {
        if (!identified) {
          if (entropy(nullptr, bootId, 8)) { delay(PollMs); continue; }
          encodeHex(bootId, 8, bootHex);
          identified = true;
        }
        if (!token[0]) {
          if (!renew(http)) { delay(PollMs); continue; }
          reportDue = true;
        }
        const Mode current = mode.load();
        char request[320], response[384];
        int status = 0;
        if (reportDue || current != lastMode || uint32_t(millis() - lastReport) >= StatusMs) {
          statusPacket[6] = static_cast<uint8_t>(current);
          unsigned char packed[32];
          size_t size = 0;
          if (!mbedtls_base64_encode(packed, sizeof packed, &size, statusPacket, sizeof statusPacket)) {
            snprintf(request, sizeof request, "{\"domain\":\"%s\",\"id\":%u,\"packets\":\"%s\"}", profile.http.domain, node, packed);
            reportDue = !http.postJson(profile.http, "/addPackets", request, token, response, sizeof response, status) || status != 202;
            if (status == 401) token[0] = 0;
            lastMode = current;
            lastReport = millis();
          }
        }
        if (token[0] && !retryRequested && (current == Mode::Quarantined || current == Mode::CommsOnly)) {
          snprintf(request, sizeof request, "{\"domain\":\"%s\",\"id\":%u,\"limit\":1,\"recoveryBoot\":\"%s\"}", profile.http.domain, node, bootHex);
          if (http.postJson(profile.http, "/commands/poll", request, token, response, sizeof response, status) &&
              status == 200 && batch.parse(response, node, 1) && batch.count == 1) {
            const auto& action = batch.packets[0];
            const uint8_t header[] = {node, node, 1, 0xc0 | 24, 8, Version};
            if (action.size == 14 && !memcmp(action.bytes, header, sizeof header) &&
                !memcmp(action.bytes + sizeof header, bootId, 8)) {
              retryRequested = true;
              retry.store(true);
            }
          }
          if (status == 401) token[0] = 0;
        }
        repair.poll(http, profile, bootHex, current, token);
      }
      delay(PollMs);
    }
  }
};

Worker* worker = nullptr;
std::unique_ptr<char[]> usbProfile;
size_t usbLength = 0;
constexpr size_t ProfileLimit = 4096;
}

namespace cloudRecovery {
void usbCommand(const char* command, uint8_t node) {
#ifdef CONFIG_TRIAL_BENCH
  if (worker && strcmp(command, "bench drop") == 0) {
    worker->repair.dropConfirmation.store(true);
    DroneConsole.println("bench drop ok");
  }
#endif
  if (strncmp(command, "enroll ", 7)) return;
  command += 7;
  Preferences prefs;
  if (!strcmp(command, "key")) {
    DeviceKey key;
    if (!prefs.begin(PreferenceSpace, false)) { DroneConsole.println("enroll error"); return; }
    const size_t length = prefs.getBytesLength("p256");
    if (length != sizeof key || prefs.getBytes("p256", &key, sizeof key) != sizeof key) {
      if (length || !makeKey(key) || prefs.putBytes("p256", &key, sizeof key) != sizeof key) {
        DroneConsole.println("enroll error"); return;
      }
    }
    char publicHex[129];
    encodeHex(key.point + 1, 64, publicHex);
    DroneConsole.printf("enroll public %u %s\n", unsigned(node), publicHex);
    return;
  }
  if (!strcmp(command, "begin")) {
    usbProfile.reset(new (std::nothrow) char[ProfileLimit]); usbLength = 0;
    DroneConsole.println(usbProfile ? "enroll ok" : "enroll error");
    return;
  }
  if (!strncmp(command, "part ", 5)) {
    const size_t length = strlen(command + 5);
    if (!usbProfile || !length || usbLength + length >= ProfileLimit) { usbProfile.reset(); DroneConsole.println("enroll error"); return; }
    memcpy(usbProfile.get() + usbLength, command + 5, length); usbLength += length;
    DroneConsole.println("enroll ok");
    return;
  }
  if (!strcmp(command, "save")) {
    std::unique_ptr<char[]> typed(new (std::nothrow) char[usbLength + 1]);
    std::unique_ptr<NodeRecoveryProfile> parsed(new (std::nothrow) NodeRecoveryProfile);
    if (usbProfile && typed) { memcpy(typed.get(), usbProfile.get(), usbLength); typed[usbLength] = 0; }
    const bool ok = usbProfile && typed && parsed && parsed->parse(typed.get()) && parsed->node == node &&
      prefs.begin(PreferenceSpace, false) && prefs.getBytesLength("p256") == sizeof(DeviceKey) &&
      prefs.putBytes("profile", usbProfile.get(), usbLength) == usbLength;
    usbProfile.reset();
    DroneConsole.println(ok ? "enroll ok" : "enroll error");
    return;
  }
  if (!strcmp(command, "clear")) {
    const bool ok = prefs.begin(PreferenceSpace, false) && prefs.clear();
    DroneConsole.println(ok ? "enroll ok" : "enroll error");
  }
}

void start(uint8_t node) {
  if (worker || !node || node == 255) return;
  Preferences prefs;
  if (!prefs.begin(PreferenceSpace, true)) return;
  const size_t size = prefs.getBytesLength("profile");
  if (!size || size >= ProfileLimit) return;
  std::unique_ptr<char[]> input(new (std::nothrow) char[size + 1]);
  std::unique_ptr<Worker> candidate(new (std::nothrow) Worker);
  if (!input || !candidate || prefs.getBytes("profile", input.get(), size) != size) return;
  input[size] = 0;
  if (!candidate->profile.parse(input.get()) || candidate->profile.node != node ||
      prefs.getBytes("p256", &candidate->key, sizeof candidate->key) != sizeof candidate->key) return;
  candidate->reason = bootRecovery::failureReason();
  candidate->phase = static_cast<uint8_t>(bootRecovery::failure().phase);
  candidate->fault = bootRecovery::failure().module;
  candidate->quarantine = bootRecovery::quarantinedModule();
  candidate->mode.store(currentMode());
  if (xTaskCreate([](void* context) { static_cast<Worker*>(context)->run(); },
      "cloud-recovery", 12288, candidate.get(), 1, nullptr) == pdPASS) worker = candidate.release();
}

void service() {
  if (!worker) return;
  const Mode mode = currentMode();
  worker->mode.store(mode);
  worker->repair.service(mode);
  if (worker->retry.exchange(false) && (mode == Mode::Quarantined || mode == Mode::CommsOnly)) {
    bootRecovery::requestRestart();
    ESP.restart();
  }
}
}
