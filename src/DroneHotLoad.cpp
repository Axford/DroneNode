#include "DroneSystem.h"
#include "droneModules/DigitalModule.h"
#include "droneModules/ManagementModule.h"
#include "droneModules/ProbeModule.h"
#include "droneModules/NMEAModule.h"
#include "droneModules/ServoModule.h"
#include "CloudTelemetryConfig.h"
#include <mbedtls/sha256.h>
#include <math.h>
#include <errno.h>
#include <new>

using configuration::HotLoadResult;

namespace {

constexpr size_t MaxFileSize = 8192;
constexpr uint8_t MaxChannel = 254;

bool hasModuleName(DroneModule &module, const char* name) {
  const auto* param = module.getParamEntryById(DRONE_MODULE_PARAM_NAME);
  const size_t length = (param->paramTypeLength & 15) + 1;
  return strlen(name) == length && !memcmp(param->data.c, name, length);
}

bool applyLogicValue(DRONE_PARAM_ENTRY &param, const char* value) {
  const auto type = (param.paramTypeLength >> 4) & 7;
  if (!(param.paramTypeLength & DRONE_LINK_MSG_WRITABLE)) return false;
  const auto size = (param.paramTypeLength & 15) + 1;
  char* end;
  for (unsigned i = 0; i < size / DRONE_LINK_MSG_TYPE_SIZES[type]; ++i) {
    if (type == DRONE_LINK_MSG_TYPE_FLOAT) {
      const float parsed = strtof(value, &end);
      if (!isfinite(parsed)) return false;
      param.data.f[i] = parsed;
    } else if (type == DRONE_LINK_MSG_TYPE_UINT8_T || type == DRONE_LINK_MSG_TYPE_UINT32_T) {
      if (*value < '0' || *value > '9') return false;
      errno = 0;
      const auto parsed = strtoul(value, &end, 10);
      if (errno || parsed > (type == DRONE_LINK_MSG_TYPE_UINT8_T ? 255u : UINT32_MAX)) return false;
      if (type == DRONE_LINK_MSG_TYPE_UINT8_T) param.data.uint8[i] = parsed;
      else param.data.uint32[i] = parsed;
    } else {
      return false;
    }
    if (end == value) return false;
    if (!*end) return i + 1 == size / DRONE_LINK_MSG_TYPE_SIZES[type];
    if (*end != ',') return false;
    value = end + 1;
  }
  return false;
}

bool applySetting(DroneModule &module, configuration::Line &line, bool digital, bool nmea, bool servo, bool logic) {
  const auto configured = module.configure(line.name, line.value);
  if (configured != configuration::Setting::Unhandled)
    return configured == configuration::Setting::Handled;
  if (!strcmp(line.name, "publish")) {
    char* remaining = line.value;
    while (remaining) {
      char* name = remaining;
      remaining = strchr(name, ',');
      if (remaining) *remaining++ = 0;
      auto* param = module.getParamEntryByName(name);
      if (!param) return false;
      param->publish = true;
    }
    return true;
  }

  auto* param = module.getParamEntryByName(line.name);
  if (!param) return false;
  const auto id = getDroneLinkMsgParam(param->paramPriority);
  if (logic && id != DRONE_MODULE_PARAM_NAME && id != DRONE_MODULE_PARAM_STATUS &&
      id != DRONE_MODULE_PARAM_INTERVAL) {
    if (!applyLogicValue(*param, line.value)) return false;
    if (auto* input = module.getSubByName(line.name)) input->received = true;
    return true;
  }
  if (nmea && id == NMEA_PARAM_PORT) {
    const char* value = line.value;
    if ((*value != '1' && *value != '2') ||
        (value[1] && strcmp(value + 1, ",255"))) return false;
    param->data.uint8[0] = *value - '0';
    param->data.uint8[1] = 255;
    return true;
  }
  if (nmea && (id == NMEA_PARAM_FIX || id == NMEA_PARAM_FOLLOWME)) {
    const char* remaining = line.value;
    for (unsigned i = 0; i < 3; ++i) {
      char* end;
      const float value = strtof(remaining, &end);
      if (end == remaining || !isfinite(value) || *end != (i == 2 ? '\0' : ',')) return false;
      param->data.f[i] = value;
      remaining = end + 1;
    }
    return true;
  }
  if (nmea && id == NMEA_PARAM_BAUD) {
    unsigned baud;
    if (!telemetry::decimal(line.value, 115200, baud) || baud < 1200) return false;
    param->data.uint32[0] = baud;
    return true;
  }
  if (!digital && !((nmea || servo || logic) && id == DRONE_MODULE_PARAM_INTERVAL) && id != DRONE_MODULE_PARAM_NAME && id != DRONE_MODULE_PARAM_STATUS) return false;
  unsigned number = 0;
  switch (id) {
    case DIGITAL_PARAM_PINS:
      if (!telemetry::decimal(line.value, 33, number) || (number != 32 && number != 33)) return false;
      break;
    case DRONE_MODULE_PARAM_NAME:
      if (!telemetry::printable(line.value, 15) || strchr(line.value, ',')) return false;
      break;
    case DRONE_MODULE_PARAM_STATUS:
      if (strcmp(line.value, "0") && strcmp(line.value, "1")) return false;
      break;
    case DRONE_MODULE_PARAM_INTERVAL:
      if (strcmp(line.value, "0") && !telemetry::decimal(line.value, 2147483647, number)) return false;
      break;
    case DIGITAL_PARAM_LIMITS: {
      char* comma;
      const float low = strtof(line.value, &comma);
      if (comma == line.value || *comma != ',') return false;
      char* end;
      const float high = strtof(comma + 1, &end);
      if (end == comma + 1 || *end || !isfinite(low) || !isfinite(high)) return false;
      param->data.f[0] = low;
      param->data.f[1] = high;
      return true;
    }
    default:
      return false;
  }
  if (strlen(line.value) > 19) return false;
  // Use the same scalar parsing as startup.
  module.setParamFromList(param, line.value);
  return true;
}

struct Candidates {
  DroneModule* modules[MaxChannel + 1] = {};
  DroneModule* retired[MaxChannel + 1] = {};
  bool proposedIds[MaxChannel + 1] = {};
  uint64_t previousPins[MaxChannel + 1] = {};
  ~Candidates() {
    for (auto* module : modules) delete module;
  }
};

} // namespace

bool DroneExecutionManager::readHotLoadConfiguration(
    const char* path, IvanLinkedList::LinkedList<AppliedSection>& sections,
    DroneModule** candidates, HotLoadResult& result, IvanLinkedList::LinkedList<LiveBinding>* bindings) {
  result = HotLoadResult::ReadFailed;
  auto file = _fs.open(path, FILE_READ);
  if (!file || file.isDirectory()) return false;
  const size_t size = file.size();
  if (!size || size > MaxFileSize) return false;
  std::unique_ptr<char[]> text(new (std::nothrow) char[size + 1]);
  if (!text) {
    result = HotLoadResult::OutOfMemory;
    return false;
  }
  if (file.read(reinterpret_cast<uint8_t*>(text.get()), size) != size ||
      memchr(text.get(), 0, size)) return false;
  file.close();
  text[size] = 0;

  result = HotLoadResult::InvalidConfiguration;
  bool seen[MaxChannel + 1] = {};
  bool seenNode = false;
  AppliedSection current = {};
  mbedtls_sha256_context hash;
  mbedtls_sha256_init(&hash);
  auto add = [&](const char* value) {
    mbedtls_sha256_update_ret(&hash, reinterpret_cast<const unsigned char*>(value), strlen(value) + 1);
  };
  auto finishSection = [&]() {
    if (!current.id) return true;
    mbedtls_sha256_finish_ret(&hash, current.digest);
    if (sections.add(current)) return true;
    result = HotLoadResult::OutOfMemory;
    return false;
  };
  bool valid = true;
  char* remaining = text.get();
  while (valid && remaining) {
    char* raw = remaining;
    remaining = strchr(raw, '\n');
    if (remaining) *remaining++ = 0;
    configuration::Line line;
    if (strlen(raw) > 2048 || !line.parse(raw, false)) {
      valid = false;
      break;
    }
    if (line.empty) continue;
    if (line.section) {
      unsigned id;
      const char* close = strchr(raw, ']');
      const char* comment = strchr(raw, ';');
      if (!close || (comment && comment < close) || !seenNode || !finishSection() ||
          !telemetry::decimal(line.value, MaxChannel, id) || seen[id]) {
        valid = false;
        break;
      }
      seen[id] = true;
      current.id = id;
      current.type = !strcmp(line.name, DIGITAL_STR_DIGITAL) ? HotType::Digital :
                     !strcmp(line.name, "Probe") ? HotType::Probe :
                     !strcmp(line.name, NMEA_STR_NMEA) ? HotType::NMEA :
                     !strcmp(line.name, SERVO_STR_SERVO) ? HotType::Servo :
                     supportsLogicHotLoad(line.name) ? HotType::Logic : HotType::StartupOnly;
      current.valid = true;
      mbedtls_sha256_starts_ret(&hash, 0);
      add(line.name);
      if (candidates && current.type != HotType::StartupOnly) {
        bool ready = false;
        if (current.type == HotType::Digital) {
          auto* module = prepareDigital(id);
          candidates[id] = module;
          ready = module && module->hotLoadReady();
        } else if (current.type == HotType::NMEA) {
          auto* module = prepareNMEA(id);
          candidates[id] = module;
          ready = module && module->hotLoadReady();
        } else if (current.type == HotType::Servo) {
          auto* module = prepareServo(id);
          candidates[id] = module;
          ready = module && module->hotLoadReady();
        } else if (current.type == HotType::Logic) {
          candidates[id] = prepareLogic(line.name, id);
          ready = candidates[id] != nullptr;
          if (ready) candidates[id]->_liveLogic = true;
        } else {
          auto* module = prepareProbe(id);
          candidates[id] = module;
          ready = module && module->hotLoadReady();
        }
        if (!ready) {
          result = HotLoadResult::OutOfMemory;
          valid = false;
        }
      }
    } else if (!current.id) {
      unsigned node;
      valid = !seenNode && !strcmp(line.name, "node") &&
              telemetry::decimal(line.value, MaxChannel, node) && node == _dmm->node();
      seenNode = valid;
    } else {
      add(line.name);
      add(line.value);
      if ((current.type == HotType::NMEA || current.type == HotType::Servo) && line.name[0] == '$') current.valid = false;
      if (candidates && candidates[current.id] && current.valid) {
        if (current.type == HotType::Logic && line.name[0] == '$') {
          LiveBinding binding{};
          binding.channel = current.id;
          current.valid = strlen(line.value) < sizeof(binding.address);
          if (current.valid) {
            strcpy(binding.input, line.name + 1);
            strcpy(binding.address, line.value);
            if (!bindings->add(binding)) {
              result = HotLoadResult::OutOfMemory;
              valid = false;
            }
          }
        } else {
          current.valid = applySetting(*candidates[current.id], line, current.type == HotType::Digital,
                                       current.type == HotType::NMEA, current.type == HotType::Servo,
                                       current.type == HotType::Logic);
        }
        if (!current.valid && (current.type == HotType::Digital || current.type == HotType::Probe))
          valid = false;
      }
    }
  }
  valid = valid && seenNode && current.id && finishSection();
  mbedtls_sha256_free(&hash);
  return valid;
}

void DroneExecutionManager::rememberBootConfiguration(const char* filename) {
  HotLoadResult result;
  _hotLoadBaselineValid = readHotLoadConfiguration(filename, _appliedSections, nullptr, result);
  if (_appliedSections.size() != _dmm->moduleCount()) _hotLoadBaselineValid = false;
}

void DroneExecutionManager::requestHotLoad(ManagementModule* requester, const char* name, size_t length) {
  if (_hotLoadRequester) return;
  if (!configuration::hotLoadPath(name, length, _hotLoadPath)) {
    requester->reportHotLoad(HotLoadResult::InvalidFilename);
    return;
  }
  _hotLoadRequester = requester;
  requester->reportHotLoad(HotLoadResult::Pending);
}

void DroneExecutionManager::processHotLoad() {
  if (!_hotLoadRequester) return;
  auto* requester = _hotLoadRequester;
  _hotLoadRequester = nullptr;
  requester->reportHotLoad(applyHotLoad(_hotLoadPath));
}

bool DroneExecutionManager::bindLiveInputs(IvanLinkedList::LinkedList<LiveBinding>& bindings,
                                            DroneModule** candidates, DroneModule** retiring) {
  for (int i = 0; i < bindings.size(); ++i) {
    auto binding = bindings.get(i);
    auto* module = candidates[binding.channel];
    if (!module) continue;
    auto* input = module->getSubByName(binding.input);
    if (!input || input->received || input->addr.node || strncmp(binding.address, "@>", 2)) return false;
    char* sourceName = binding.address + 2;
    char* parameterName = strchr(sourceName, '.');
    if (!parameterName || parameterName == sourceName || !parameterName[1]) return false;
    *parameterName++ = 0;
    DroneModule* source = nullptr;
    unsigned id;
    if (*sourceName >= '0' && *sourceName <= '9') {
      if (!telemetry::decimal(sourceName, MaxChannel, id) || retiring[id] || candidates[id])
        return false;
      source = _dmm->getModuleById(id);
    } else {
      for (uint8_t index = 0; index < _dmm->moduleCount(); ++index) {
        auto* current = _dmm->getModuleByIndex(index);
        if (!hasModuleName(*current, sourceName)) continue;
        if (source || retiring[current->id()] || candidates[current->id()]) return false;
        source = current;
      }
      for (unsigned index = 1; index <= MaxChannel; ++index)
        if (candidates[index] && hasModuleName(*candidates[index], sourceName)) return false;
    }
    if (!source || (source->_liveLogic && source->liveObservationOnly())) return false;
    DRONE_PARAM_ENTRY* param;
    if (*parameterName >= '0' && *parameterName <= '9') {
      if (!telemetry::decimal(parameterName, 63, id)) return false;
      param = source->getParamEntryById(id);
    } else {
      param = source->getParamEntryByName(parameterName);
    }
    if (!param || !param->publish || (param->paramTypeLength & 127) !=
                                   (input->param.paramTypeLength & 127)) return false;
    module->setSubAddr(input, _dmm->node(), source->id(), getDroneLinkMsgParam(param->paramPriority));
  }
  return true;
}

HotLoadResult DroneExecutionManager::applyHotLoad(const char* path) {
  if (!_hotLoadBaselineValid) return HotLoadResult::RestartRequired;
  HotLoadResult result;
  std::unique_ptr<Candidates> work(new (std::nothrow) Candidates);
  if (!work) return HotLoadResult::OutOfMemory;
  auto &candidates = work->modules;
  auto &retired = work->retired;
  auto &proposedIds = work->proposedIds;
  auto &previousPins = work->previousPins;
  IvanLinkedList::LinkedList<AppliedSection> proposed;
  IvanLinkedList::LinkedList<LiveBinding> bindings;
  if (!readHotLoadConfiguration(path, proposed, candidates, result, &bindings)) return result;
  for (int i = 0; i < proposed.size(); ++i) {
    auto section = proposed.get(i);
    proposedIds[section.id] = true;
    auto* previous = _dmm->getModuleById(section.id);
    if (!previous) {
      if (section.type == HotType::StartupOnly) return HotLoadResult::RestartRequired;
      if (!section.valid) return HotLoadResult::InvalidConfiguration;
      continue;
    }
    for (int j = 0; j < _appliedSections.size(); ++j) {
      auto active = _appliedSections.get(j);
      if (active.id != section.id) continue;
      if (!memcmp(active.digest, section.digest, sizeof(active.digest))) {
        delete candidates[section.id];
        candidates[section.id] = nullptr;
      } else if (active.type == section.type && active.type != HotType::StartupOnly) {
        // Preserve consumer binding and publication semantics for this first lifecycle.
        if (_dlm->hasLocalConsumer(section.id, previous)) return HotLoadResult::InvalidConfiguration;
        if (active.type == HotType::Logic) {
          auto* oldType = previous->getParamEntryById(DRONE_MODULE_PARAM_TYPE);
          auto* newType = candidates[section.id]->getParamEntryById(DRONE_MODULE_PARAM_TYPE);
          if (oldType->paramTypeLength != newType->paramTypeLength ||
              memcmp(oldType->data.c, newType->data.c, (oldType->paramTypeLength & 15) + 1))
            return HotLoadResult::RestartRequired;
        }
        if (active.type == HotType::Servo && !active.valid) return HotLoadResult::RestartRequired;
        if (active.type == HotType::NMEA &&
            (!active.valid || !static_cast<NMEAModule*>(previous)->hotLoadEligible()))
          return HotLoadResult::RestartRequired;
        if (!section.valid) return HotLoadResult::InvalidConfiguration;
        retired[section.id] = previous;
      } else {
        return HotLoadResult::RestartRequired;
      }
      break;
    }
  }
  for (int i = 0; i < _appliedSections.size(); ++i) {
    auto section = _appliedSections.get(i);
    if (proposedIds[section.id]) continue;
    if (section.type == HotType::StartupOnly) return HotLoadResult::RestartRequired;
    auto* module = _dmm->getModuleById(section.id);
    if (_dlm->hasLocalConsumer(section.id, module)) return HotLoadResult::InvalidConfiguration;
    if (section.type == HotType::Servo && !section.valid) return HotLoadResult::RestartRequired;
    if (section.type == HotType::NMEA &&
        (!section.valid || !static_cast<NMEAModule*>(module)->hotLoadEligible()))
      return HotLoadResult::RestartRequired;
    retired[section.id] = module;
  }

  HotType types[MaxChannel + 1] = {};
  for (int i = 0; i < _appliedSections.size(); ++i) {
    const auto section = _appliedSections.get(i);
    types[section.id] = section.type;
  }
  for (int i = 0; i < proposed.size(); ++i) {
    const auto section = proposed.get(i);
    types[section.id] = section.type;
  }
  uint64_t claimed = 0;
  for (unsigned id = 1; id <= MaxChannel; ++id) {
    if (retired[id]) {
      for (uint8_t pin = 0; pin < DRONE_SYSTEM_PINS; ++pin)
        if (_ds->_pins[pin].module == retired[id]) previousPins[id] |= uint64_t{1} << pin;
      if ((types[id] == HotType::Digital || types[id] == HotType::Servo) && (previousPins[id] & ~(uint64_t{3} << 32)))
        return HotLoadResult::RestartRequired;
    }
    if (!candidates[id]) continue;
    if (types[id] == HotType::NMEA) {
      const auto port = candidates[id]->getParamEntryById(NMEA_PARAM_PORT)->data.uint8[0];
      auto* owner = port < DRONE_SYSTEM_SERIAL_PORTS ? _ds->_serialPorts[port].module : nullptr;
      auto* retiring = owner && owner->id() <= MaxChannel ? retired[owner->id()] : nullptr;
      if (!_ds->secondarySerialAvailable(port, retiring)) return HotLoadResult::ResourceConflict;
    }
    const uint64_t requested = hotLoadPins(candidates[id], types[id]);
    if (!requested && types[id] != HotType::Logic) return HotLoadResult::InvalidConfiguration;
    if (claimed & requested) return HotLoadResult::ResourceConflict;
    claimed |= requested;
    for (uint8_t pin = 0; pin < DRONE_SYSTEM_PINS; ++pin) {
      if (!(requested & (uint64_t{1} << pin))) continue;
      const auto &entry = _ds->_pins[pin];
      const auto capability = types[id] == HotType::Servo ?
          DRONE_SYSTEM_PIN_CAP_OUTPUT : DRONE_SYSTEM_PIN_CAP_INPUT;
      if (!(entry.capabilities & capability)) return HotLoadResult::ResourceConflict;
      if (entry.state == DRONE_SYSTEM_PIN_STATE_AVAILABLE) continue;
      if (!entry.module || entry.strip || entry.module->id() > MaxChannel || retired[entry.module->id()] != entry.module)
        return HotLoadResult::ResourceConflict;
    }
  }

  // Unresolved inputs can still bind to a channel that is about to change.
  // Wait for the address queue to settle before changing the module set.
  bool changed = false;
  for (unsigned id = 1; id <= MaxChannel; ++id)
    changed |= retired[id] || candidates[id];
  if (!changed) return HotLoadResult::Applied;
  if (_addressQueue.size() || !bindLiveInputs(bindings, candidates, retired))
    return HotLoadResult::InvalidConfiguration;
  for (auto* module : candidates)
    if (module && module->_liveLogic && !module->validateLiveConfiguration())
      return HotLoadResult::InvalidConfiguration;

  xSemaphoreTake(_ds->_moduleChangeMutex, portMAX_DELAY);
  // Allocate registry entries while the previous modules still own their pins.
  bool prepared = true;
  for (auto* module : candidates) {
    if (!module) continue;
    if (!_dmm->registerModule(module) || !_dlm->subscribe(module->id(), module, DRONE_LINK_PARAM_ALL)) {
      prepared = false;
      break;
    }
    if (!module->_liveLogic) continue;
    for (uint8_t index = 0; auto* input = module->subscription(index); ++index) {
      const auto& addr = input->addr;
      if (addr.node && !_dlm->subscribe(addr.node, addr.channel, module, addr.paramPriority, true)) {
        prepared = false;
        break;
      }
    }
    if (!prepared) break;
  }
  bool started = prepared;
  if (prepared) {
    for (unsigned id = 1; id <= MaxChannel; ++id)
      if (retired[id]) releaseHotLoadPins(retired[id], types[id]);
    for (unsigned id = 1; id <= MaxChannel; ++id) {
      auto* module = candidates[id];
      if (!module) continue;
      module->setup();
      if (types[id] == HotType::NMEA && static_cast<NMEAModule*>(module)->_activePort == 255)
        started = false;
      const auto requested = hotLoadPins(module, types[id]);
      for (uint8_t pin = 0; pin < DRONE_SYSTEM_PINS; ++pin) {
        if ((requested & (uint64_t{1} << pin)) && _ds->_pins[pin].module != module)
          started = false;
      }
      if (!started) break;
    }
  }
  if (!started) {
    for (unsigned id = 1; id <= MaxChannel; ++id) {
      auto* module = candidates[id];
      if (!module) continue;
      _dmm->unregisterModule(module);
      _dlm->detachModule(module);
      releaseHotLoadPins(module, types[id]);
    }
    result = prepared ? HotLoadResult::StartFailed : HotLoadResult::OutOfMemory;
    if (prepared) {
      for (unsigned id = 1; id <= MaxChannel; ++id) {
        if (retired[id] && !restoreHotLoadPins(retired[id], types[id], previousPins[id])) {
          retired[id]->disable();
          _hotLoadBaselineValid = false;
          result = HotLoadResult::RecoveryFailed;
        }
      }
    }
  } else {
    for (unsigned id = 1; id <= MaxChannel; ++id) {
      if (retired[id] || candidates[id]) _dlm->discardLocalChannelMessages(id);
      if (!retired[id]) continue;
      retired[id]->doShutdown();
      _dmm->unregisterModule(retired[id]);
      _dlm->detachModule(retired[id]);
      delete retired[id];
    }
    _appliedSections.swap(proposed);
    for (auto &module : candidates) module = nullptr;
    result = HotLoadResult::Applied;
  }
  xSemaphoreGive(_ds->_moduleChangeMutex);
  return result;
}


uint64_t DroneExecutionManager::hotLoadPins(DroneModule* module, HotType type) {
  if (type == HotType::Logic) return 0;
  if (type == HotType::Probe) return static_cast<ProbeModule*>(module)->requestedInputs();
  if (type == HotType::NMEA) {
    const auto port = module->getParamEntryById(NMEA_PARAM_PORT)->data.uint8[0];
    if (port == 1) return (uint64_t{1} << PIN_SERIAL1_RX) | (uint64_t{1} << PIN_SERIAL1_TX);
    if (port == 2) return (uint64_t{1} << PIN_SERIAL2_RX) | (uint64_t{1} << PIN_SERIAL2_TX);
    return 0;
  }
  const auto pin = type == HotType::Servo ?
      module->getParamEntryById(SERVO_PARAM_PINS)->data.uint8[0] :
      static_cast<DigitalModule*>(module)->trialPin();
  return pin == 32 || pin == 33 ? uint64_t{1} << pin : 0;
}

void DroneExecutionManager::releaseHotLoadPins(DroneModule* module, HotType type) {
  if (type == HotType::Logic) return;
  if (type == HotType::Probe) static_cast<ProbeModule*>(module)->releaseInputs();
  else if (type == HotType::NMEA) static_cast<NMEAModule*>(module)->releasePrimary();
  else if (type == HotType::Servo) {
    auto* servo = static_cast<ServoModule*>(module);
    servo->_localControl = true;
    servo->doShutdown();
  } else static_cast<DigitalModule*>(module)->releaseHotLoadPin();
}

bool DroneExecutionManager::restoreHotLoadPins(DroneModule* module, HotType type, uint64_t pins) {
  if (!pins) return true;
  if (type == HotType::Probe) return static_cast<ProbeModule*>(module)->restoreInputs(pins);
  if (type == HotType::NMEA) {
    auto* nmea = static_cast<NMEAModule*>(module);
    const uint8_t port = pins & (uint64_t{1} << PIN_SERIAL1_RX) ? 1 : 2;
    return nmea->startPrimary(port, nmea->_activeBaud);
  }
  if (type == HotType::Servo)
    return static_cast<ServoModule*>(module)->reserveOutput(pins & (uint64_t{1} << 32) ? 32 : 33);
  return static_cast<DigitalModule*>(module)->restoreHotLoadPin(pins & (uint64_t{1} << 32) ? 32 : 33);
}


void DroneExecutionManager::runServoConsole(const char* command, Print &output) {
  char* end;
  const auto id = strtoul(command, &end, 10);
  if (end == command || *end != ' ' || !id || id > MaxChannel) {
    output.println("servo error=invalid-command");
    return;
  }
  for (int i = 0; i < _appliedSections.size(); ++i) {
    const auto section = _appliedSections.get(i);
    if (section.id != id || section.type != HotType::Servo) continue;
    auto* module = _dmm->getModuleById(id);
    if (module) {
      static_cast<ServoModule*>(module)->consoleControl(end + 1, output);
      return;
    }
  }
  output.println("servo error=unavailable");
}
