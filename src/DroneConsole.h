#pragma once
#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "SerialTransport.h"

// The application's Serial0 Print sink. Hardware control/input stays on Serial.
class DiagnosticConsole : public Print {
 public:
  void begin();
  void telemetry(bool enabled);
  bool telemetryActive() const {return framed.load();}
  bool sendMesh(const uint8_t* bytes);
  void service();
  using Print::write;
  size_t write(uint8_t byte) override {return write(&byte,1);}
  size_t write(const uint8_t* bytes,size_t size) override;
 private:
  StaticSemaphore_t mutexStorage;
  SemaphoreHandle_t mutex=nullptr;
  std::atomic<bool> contended{false};
  std::atomic<bool> framed{false};
  serialTransport::Writer writer;
  // Sink uses a separate adapter because Print::write is diagnostic input.
  class Wire : public serialTransport::Sink {
   public:
    size_t capacity() override {return Serial.availableForWrite();}
    size_t write(const uint8_t* bytes,size_t size) override {return Serial.write(bytes,size);}
  } wire;
  void reportContention();
};
extern DiagnosticConsole DroneConsole;
