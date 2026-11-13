#include "ProbeModule.h"

#include "../CloudTelemetryConfig.h"
#include "../DroneSystem.h"
#include <driver/adc.h>
#include <driver/gpio.h>
#include <esp_adc_cal.h>
#include <hal/gpio_ll.h>

namespace {
struct Pin {
  uint8_t gpio;
  const char* result;
  const char* baud;
  adc1_channel_t adc;
};

constexpr Pin pins[] = {
  {4, "gpio4", "baud4", ADC1_CHANNEL_MAX},
  {13, "gpio13", "baud13", ADC1_CHANNEL_MAX},
  {14, "gpio14", "baud14", ADC1_CHANNEL_MAX},
  {15, "gpio15", "baud15", ADC1_CHANNEL_MAX},
  {16, "gpio16", "baud16", ADC1_CHANNEL_MAX},
  {25, "gpio25", "baud25", ADC1_CHANNEL_MAX},
  {26, "gpio26", "baud26", ADC1_CHANNEL_MAX},
  {32, "gpio32", "baud32", ADC1_CHANNEL_4},
  {33, "gpio33", "baud33", ADC1_CHANNEL_5},
  {34, "gpio34", "baud34", ADC1_CHANNEL_6},
  {35, "gpio35", "baud35", ADC1_CHANNEL_7},
};

constexpr uint8_t resultBase = 10;
constexpr uint8_t baudBase = 46;
constexpr uint32_t digitalWindow = 20000;
constexpr uint32_t serialWindow = 100000;
constexpr uint32_t edgeLimit = 8192;
constexpr uint32_t minimumInterval = 200;
constexpr uint32_t maximumInterval = 60000;

bool reached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}
}

ProbeModule::ProbeModule(uint8_t id, DroneSystem* system) : DroneModule(id, system) {
  static_assert(pinCount == sizeof(pins) / sizeof(pins[0]), "Probe pin table size");
  setTypeName(F("Probe"));
  _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 500;
  initParams(2 * pinCount);
  if (!_params) return;

  for (uint8_t i = 0; i < pinCount; ++i) {
    auto& result = _params[i];
    result.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, resultBase + pins[i].gpio);
    setParamName(FPSTR(pins[i].result), &result);
    result.paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
    auto& baud = _params[pinCount + i];
    baud.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, baudBase + i);
    setParamName(FPSTR(pins[i].baud), &baud);
    baud.paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
  }
}

ProbeModule::~ProbeModule() {
  doShutdown();
}

configuration::Setting ProbeModule::configure(const char* name, const char* value) {
  using configuration::Setting;
  if (strcmp(name, "interval") == 0) {
    unsigned interval = 0;
    if (!telemetry::decimal(value, maximumInterval, interval) || interval < minimumInterval) return Setting::Invalid;
    auto& setting = _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E];
    setting.data.uint32[0] = interval;
    setting.changed = true;
    return Setting::Handled;
  }
  // Result and baud parameters are observations, including in saved configuration.
  if (strncmp(name, "gpio", 4) == 0 || strncmp(name, "baud", 4) == 0) return Setting::Invalid;
  if (strncmp(name, "pin", 3) != 0) return Setting::Unhandled;
  unsigned gpio = 0;
  if (!telemetry::decimal(name + 3, 35, gpio)) return Setting::Invalid;
  for (uint8_t i = 0; i < pinCount; ++i) {
    if (pins[i].gpio != gpio) continue;
    auto& input = _inputs[i];
    if (input.mode != Unselected) return Setting::Invalid;
    if (strcmp(value, "digital") == 0) {
      input.mode = Digital;
      return Setting::Handled;
    }
    if (strcmp(value, "analog") == 0 || strcmp(value, "analog,voltage") == 0) {
      if (pins[i].adc == ADC1_CHANNEL_MAX) return Setting::Invalid;
      input.mode = Analog;
      input.estimateVoltage = strchr(value, ',') != nullptr;
      return Setting::Handled;
    }
    unsigned baud = 0;
    if (strncmp(value, "serial,", 7) == 0 && telemetry::decimal(value + 7, 38400, baud) && baud >= 1200) {
      input.mode = Serial;
      input.baud = baud;
      return Setting::Handled;
    }
    return Setting::Invalid;
  }
  return Setting::Invalid;
}

void ProbeModule::saveExtraConfiguration(File* file) {
  for (uint8_t i = 0; i < pinCount; ++i) {
    const auto& input = _inputs[i];
    if (input.mode == Unselected) continue;
    file->printf("  pin%u = ", pins[i].gpio);
    if (input.mode == Serial) file->printf("serial, %lu\n", static_cast<unsigned long>(input.baud));
    else file->println(input.mode == Digital ? "digital" : input.estimateVoltage ? "analog, voltage" : "analog");
  }
}

void ProbeModule::setup() {
  DroneModule::setup();
  if (!_params) {
    setError(1);
    return;
  }
  for (uint8_t i = 0; i < pinCount; ++i) {
    auto& input = _inputs[i];
    // Discovery replaces retained values even for pins removed from a later configuration.
    _params[i].publish = true;
    auto& baud = _params[pinCount + i];
    baud.publish = true;
    baud.data.uint32[0] = input.baud;
    if (input.mode == Unselected) continue;
    const auto& pin = pins[i];
    uint8_t capabilities = DRONE_SYSTEM_PIN_CAP_INPUT;
    if (input.mode == Analog) capabilities |= DRONE_SYSTEM_PIN_CAP_ANALOG;
    input.claimed = isEnabled() && !_error && _ds->requestPin(pin.gpio, capabilities, this);
    if (input.claimed) pinMode(pin.gpio, INPUT);
    const uint8_t owner = input.claimed ? 0 : _ds->pinOwnerChannel(pin.gpio);
    setResult(i, input.claimed ? Waiting : Unavailable, &owner, owner ? 1 : 0);
  }
  _nextStart = millis();
}

void ProbeModule::disable() {
  stopCapture();
  DroneModule::disable();
}

void ProbeModule::onParamWrite(DRONE_PARAM_ENTRY* param) {
  auto& interval = _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E];
  if (param == &interval) {
    interval.data.uint32[0] = constrain(interval.data.uint32[0], minimumInterval, maximumInterval);
    interval.paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
    _nextStart = millis() + interval.data.uint32[0];
  }
  DroneModule::onParamWrite(param);
}

void ProbeModule::doShutdown() {
  stopCapture();
  for (uint8_t i = 0; i < pinCount; ++i) {
    if (!_inputs[i].claimed) continue;
    _ds->releasePin(pins[i].gpio, this);
    _inputs[i].claimed = false;
  }
  DroneModule::doShutdown();
}

bool ProbeModule::readyToLoop() {
  return _setupDone && isEnabled() && !_error &&
    (_active >= 0 || reached(millis(), _nextStart));
}

void ProbeModule::loop() {
  DroneModule::loop();
  if (_active >= 0) {
    if (micros() - _started >= _window) finishCapture();
    else drainEdges();
    return;
  }

  const uint32_t now = millis();
  const uint32_t interval = constrain(_mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0],
                                      minimumInterval, maximumInterval);
  // Advance past any missed slots without starting a catch-up burst.
  _nextStart = now + interval;
  for (uint8_t scanned = 0; scanned < pinCount; ++scanned) {
    const uint8_t index = _cursor;
    _cursor = (_cursor + 1) % pinCount;
    const auto& input = _inputs[index];
    if (!input.claimed) continue;
    if (input.mode == Analog) {
      sampleAnalog(index);
    } else {
      _active = index;
      _capturePin = pins[index].gpio;
      _serialCapture = input.mode == Serial;
      _window = _serialCapture ? serialWindow : digitalWindow;
      _read = _write = 0;
      _count = 0;
      _incomplete = false;
      _level = digitalRead(_capturePin) != 0;
      if (_serialCapture) _decoder.reset(input.baud);
      _started = micros();
      const auto gpio = static_cast<gpio_num_t>(_capturePin);
      const auto service = gpio_install_isr_service(0);
      if ((service != ESP_OK && service != ESP_ERR_INVALID_STATE) ||
          gpio_set_intr_type(gpio, GPIO_INTR_ANYEDGE) != ESP_OK ||
          gpio_isr_handler_add(gpio, onEdge, this) != ESP_OK) {
        _active = -1;
        setResult(index, ReadFailed, nullptr, 0, true);
      }
    }
    return;
  }
}

void IRAM_ATTR ProbeModule::onEdge(void* context) {
  auto& probe = *static_cast<ProbeModule*>(context);
  const auto pin = static_cast<gpio_num_t>(probe._capturePin);
  const uint32_t time = micros();
  portENTER_CRITICAL_ISR(&probe._edgeLock);
  if (time - probe._started >= probe._window) {
    gpio_ll_intr_disable(&GPIO, pin);
  } else if (++probe._count > edgeLimit) {
    probe._incomplete = true;
    gpio_ll_intr_disable(&GPIO, pin);
  } else {
    probe._level = gpio_ll_get_level(&GPIO, pin) != 0;
    if (probe._serialCapture) {
      const uint16_t next = (probe._write + 1) % queueSize;
      if (next == probe._read) {
        probe._incomplete = true;
        gpio_ll_intr_disable(&GPIO, pin);
      } else {
        probe._edges[probe._write] = {time, probe._level};
        probe._write = next;
      }
    }
  }
  portEXIT_CRITICAL_ISR(&probe._edgeLock);
}

void ProbeModule::drainEdges() {
  if (!_serialCapture) return;
  for (uint16_t i = 0; i < queueSize; ++i) {
    portENTER_CRITICAL(&_edgeLock);
    if (_read == _write) {
      portEXIT_CRITICAL(&_edgeLock);
      return;
    }
    const Edge edge = _edges[_read];
    _read = (_read + 1) % queueSize;
    portEXIT_CRITICAL(&_edgeLock);
    _decoder.edge(edge.time, edge.high);
  }
}

void ProbeModule::stopCapture() {
  if (_active < 0) return;
  gpio_isr_handler_remove(static_cast<gpio_num_t>(_capturePin));
  portENTER_CRITICAL(&_edgeLock);
  portEXIT_CRITICAL(&_edgeLock);
  _active = -1;
}

void ProbeModule::finishCapture() {
  const uint8_t index = _active;
  stopCapture();
  uint8_t data[15] = {};
  if (_serialCapture) {
    drainEdges();
    if (!_incomplete) _decoder.advance(_started + _window);
    const uint8_t size = _decoder.copyTail(data);
    const Status status = _incomplete ? Incomplete : size ? Complete : _count ? Undecoded : Quiet;
    setResult(index, status, data, size, true);
  } else {
    data[0] = _incomplete ? digitalRead(_capturePin) != 0 : _level;
    const uint16_t count = min(_count, edgeLimit);
    const uint16_t durationMs = _window / 1000;
    data[1] = count;
    data[2] = count >> 8;
    data[3] = durationMs;
    data[4] = durationMs >> 8;
    setResult(index, _incomplete ? Incomplete : Complete, data, 5, true);
  }
}

void ProbeModule::sampleAnalog(uint8_t index) {
  const auto channel = pins[index].adc;
  if (adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK ||
      adc1_config_channel_atten(channel, ADC_ATTEN_DB_11) != ESP_OK) {
    setResult(index, ReadFailed, nullptr, 0, true);
    return;
  }
  const int raw = adc1_get_raw(channel);
  if (raw < 0 || raw > 4095) {
    setResult(index, ReadFailed, nullptr, 0, true);
    return;
  }
  uint8_t data[4] = {static_cast<uint8_t>(raw), static_cast<uint8_t>(raw >> 8)};
  if (_inputs[index].estimateVoltage) {
    esp_adc_cal_characteristics_t calibration;
    if (esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &calibration)
        != ESP_ADC_CAL_VAL_DEFAULT_VREF) {
      const uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &calibration);
      if (mv >= 150 && mv <= 2450) {
        data[2] = mv;
        data[3] = mv >> 8;
        setResult(index, Complete, data, 4, true);
        return;
      }
    }
  }
  setResult(index, Complete, data, 2, true);
}

void ProbeModule::setResult(uint8_t index, Status status, const uint8_t* data, uint8_t size, bool publish) {
  auto& result = _params[index];
  result.data.uint8[0] = (_inputs[index].mode << 4) | status;
  if (size) memcpy(result.data.uint8 + 1, data, size);
  result.paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, size + 1);
  if (publish) publishParamEntry(&result);
}
