#include "I2cDiagnostic.h"
#include "DroneWire.h"
#include <esp32-hal-i2c.h>
#include <esp_system.h>
#include <freertos/semphr.h>

namespace DroneWire {
namespace {
using i2cDiagnostic::Job;
using i2cDiagnostic::Status;

constexpr uint32_t transactionMs = 8;
StaticSemaphore_t jobMutexStorage;
SemaphoreHandle_t jobMutex;
Job job;
uint32_t completedAt;

const char* label(Status status) {
  switch (status) {
    case Status::Complete: return "complete";
    case Status::SelectionFailed: return "selection-failed";
    case Status::Nack: return "nack";
    case Status::Timeout: return "timeout";
    case Status::ShortRead: return "short-read";
    case Status::BusError: return "bus-error";
    case Status::NoMultiplexer: return "no-multiplexer";
    default: return "pending";
  }
}

Status transferStatus(esp_err_t error) {
  if (error == ESP_OK) return Status::Complete;
  if (error == ESP_FAIL) return Status::Nack;
  if (error == ESP_ERR_TIMEOUT) return Status::Timeout;
  return Status::BusError;
}

struct Bus {
  uint8_t mux;
  bool hasMux() const { return mux != 0; }

  bool select(uint8_t channel) {
    const uint8_t mask = 1 << channel;
    uint8_t actual = 0;
    size_t received = 0;
    // Invalidate the ordinary selector's cache even when selection fails.
    invalidateChannel();
    return i2cWrite(0, mux, &mask, 1, transactionMs) == ESP_OK &&
      i2cRead(0, mux, &actual, 1, transactionMs, &received) == ESP_OK && received == 1 && actual == mask;
  }

  Status ping(uint8_t address) {
    return transferStatus(i2cWrite(0, address, nullptr, 0, transactionMs));
  }

  Status read(uint8_t address, uint8_t reg, uint8_t length, uint8_t* bytes, uint8_t& received) {
    size_t count = 0;
    const auto result = transferStatus(i2cWriteReadNonStop(0, address, &reg, 1, bytes, length, transactionMs, &count));
    received = count;
    return result == Status::Complete && count != length ? Status::ShortRead : result;
  }
};

bool number(AsyncWebServerRequest* request, const char* key, uint32_t maximum, uint32_t& value) {
  if (!request->hasParam(key)) return false;
  const auto& text = request->getParam(key)->value();
  if (!text.length() || text.length() > 10) return false;
  value = 0;
  for (unsigned i = 0; i < text.length(); ++i) {
    const char digit = text[i];
    if (digit < '0' || digit > '9') return false;
    const unsigned n = digit - '0';
    if (n > maximum || value > (maximum - n) / 10) return false;
    value = value * 10 + n;
  }
  return true;
}

void send(AsyncWebServerRequest* request, const Job& result) {
  auto* response = request->beginResponseStream("application/json");
  if (!response) {
    request->send(503);
    return;
  }
  response->addHeader("Cache-Control", "no-store");
  response->setCode(result.done ? 200 : 202);
  response->printf("{\"schemaVersion\":1,\"node\":%u,\"job\":%lu", result.node, static_cast<unsigned long>(result.id));
  if (result.done && result.read) {
    response->printf(",\"channel\":%u,\"address\":%u,\"register\":%u,\"length\":%u,\"status\":\"%s\",\"bytes\":[",
      result.channel, result.address, result.reg, result.length, label(result.status[result.channel]));
    for (uint8_t i = 0; i < result.received; ++i) response->printf("%s%u", i ? "," : "", result.bytes[i]);
    response->print("]");
  } else if (result.done) {
    response->print(",\"channels\":[");
    for (uint8_t channel = 0; channel < 8; ++channel) {
      response->printf("%s{\"channel\":%u,\"status\":\"%s\",\"addresses\":[", channel ? "," : "", channel, label(result.status[channel]));
      bool first = true;
      for (unsigned address = i2cDiagnostic::firstAddress; address <= i2cDiagnostic::lastAddress; ++address) {
        if (!result.acknowledged(channel, address)) continue;
        response->printf("%s%u", first ? "" : ",", address);
        first = false;
      }
      response->print("]}");
    }
    response->print("]");
  }
  response->print("}");
  request->send(response);
}
}

void mountDiagnostic(AsyncWebServer& server, uint8_t node) {
  jobMutex = xSemaphoreCreateMutexStatic(&jobMutexStorage);
  server.on("/i2cDiagnostic", HTTP_POST, [node](AsyncWebServerRequest* request) {
    uint32_t target, channel, address, reg, length;
    if (!number(request, "node", 254, target) || target != node || !request->hasParam("op")) {
      request->send(400);
      return;
    }
    const auto& op = request->getParam("op")->value();
    Job next;
    next.node = node;
    if (op == "register") {
      if (!number(request, "channel", 7, channel) || !number(request, "address", 127, address) ||
          !number(request, "register", 255, reg) || !number(request, "length", 4, length) ||
          !i2cDiagnostic::registerAllowed(address, reg, length)) {
        request->send(400);
        return;
      }
      next.read = true;
      next.channel = channel;
      next.address = address;
      next.reg = reg;
      next.length = length;
    } else if (op != "scan") {
      request->send(400);
      return;
    }
    if (xSemaphoreTake(jobMutex, 0) != pdTRUE) { request->send(409); return; }
    if (job.id && (!job.done || millis() - completedAt < 1200)) {
      xSemaphoreGive(jobMutex);
      request->send(409);
      return;
    }
    do { next.id = esp_random(); } while (!next.id || next.id == job.id);
    job = next;
    xSemaphoreGive(jobMutex);
    send(request, next);
  });
  server.on("/i2cDiagnostic", HTTP_GET, [](AsyncWebServerRequest* request) {
    uint32_t id;
    if (!number(request, "job", UINT32_MAX, id) || !id) { request->send(400); return; }
    if (xSemaphoreTake(jobMutex, pdMS_TO_TICKS(50)) != pdTRUE) { request->send(503); return; }
    const Job result = job;
    xSemaphoreGive(jobMutex);
    if (id != result.id) { request->send(404); return; }
    send(request, result);
  });
}

void serviceDiagnostic() {
  if (!jobMutex || xSemaphoreTake(jobMutex, 0) != pdTRUE) return;
  if (job.id && !job.done) {
    const uint8_t version = getMultiplexerVersion();
    Bus bus{static_cast<uint8_t>(version == 1 ? TCAADDR_V1 : version == 2 ? TCAADDR_V2 : 0)};
    job.step(bus);
    if (job.done) completedAt = millis();
  }
  xSemaphoreGive(jobMutex);
}
}
