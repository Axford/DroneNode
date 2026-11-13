/*
@type          Probe
@inherits      Drone
@category      Input
@description   Inspects explicitly selected free inputs for wiring diagnostics.

@config >>>
[Probe = 3]
  interval = 500
  pin13 = serial, 9600
  pin32 = digital
  pin34 = analog
<<<
*/
#ifndef DRONE_PROBE_MODULE_H
#define DRONE_PROBE_MODULE_H

#include "../DroneModule.h"
#include "ProbeSerial.h"

class ProbeModule : public DroneModule {
public:
  ProbeModule(uint8_t id, DroneSystem* system);
  ~ProbeModule() override;
  configuration::Setting configure(const char* name, const char* value) override;
  void saveExtraConfiguration(File* file) override;
  void setup() override;
  void disable() override;
  void onParamWrite(DRONE_PARAM_ENTRY* param) override;
  void doShutdown() override;
  bool readyToLoop() override;
  void loop() override;

private:
  friend class DroneExecutionManager;
  ProbeModule(uint8_t id, DroneSystem* system, Registration registration);
  void releaseInputs();
  bool restoreInputs(uint64_t mask);
  uint64_t requestedInputs() const;
  bool hotLoadReady() const { return _mgmtParams && _params; }
  enum Mode : uint8_t { Unselected, Digital, Analog, Serial };
  enum Status : uint8_t { Waiting, Complete, Quiet, Undecoded, Incomplete, Unavailable, ReadFailed };
  static constexpr uint8_t pinCount = 11;
  static constexpr uint16_t queueSize = 256;
  struct Input {
    Mode mode = Unselected;
    uint32_t baud = 0;
    bool estimateVoltage = false;
    bool claimed = false;
  };
  struct Edge {
    uint32_t time;
    bool high;
  };
  Input _inputs[pinCount];
  uint8_t _cursor = 0;
  int8_t _active = -1;
  uint32_t _nextStart = 0;
  uint32_t _started = 0;
  uint32_t _window = 0;
  uint8_t _capturePin = 0;
  bool _serialCapture = false;
  portMUX_TYPE _edgeLock = portMUX_INITIALIZER_UNLOCKED;
  Edge _edges[queueSize];
  uint16_t _read = 0;
  uint16_t _write = 0;
  uint32_t _count = 0;
  bool _incomplete = false;
  bool _level = false;
  bool _interruptServiceReady = false;
  probe::SerialDecoder _decoder;

  static void IRAM_ATTR onEdge(void* context);
  void drainEdges();
  void stopCapture();
  void finishCapture();
  void startAnalog(uint8_t index);
  void sampleAnalog();
  void finishAnalog();
  uint32_t _nextAnalogSample = 0;
  uint32_t _analogSum = 0;
  uint16_t _analogMin = 0;
  uint16_t _analogMax = 0;
  uint8_t _analogCount = 0;
  void setResult(uint8_t index, Status status, const uint8_t* data = nullptr, uint8_t size = 0,
                 bool publish = false);
};

#endif
