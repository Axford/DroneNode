#include "DroneSystem.h"
#include "droneModules/DigitalModule.h"
#include "droneModules/ManagementModule.h"
#include "droneModules/ProbeModule.h"
#include "CloudTelemetryConfig.h"
#include <mbedtls/sha256.h>
#include <math.h>
#include <new>

using configuration::HotLoadResult;

namespace {

constexpr size_t MaxFileSize = 8192;
constexpr uint8_t MaxChannel = 254;

bool applySetting(DroneModule &module, configuration::Line &line, bool digital) {
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
  if (!digital && id != DRONE_MODULE_PARAM_NAME && id != DRONE_MODULE_PARAM_STATUS) return false;
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
    DroneModule** candidates, HotLoadResult& result) {
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
                     !strcmp(line.name, "Probe") ? HotType::Probe : HotType::StartupOnly;
      mbedtls_sha256_starts_ret(&hash, 0);
      add(line.name);
      if (candidates && current.type != HotType::StartupOnly) {
        bool ready = false;
        if (current.type == HotType::Digital) {
          auto* module = createDigital(id, true);
          candidates[id] = module;
          ready = module && module->hotLoadReady();
        } else {
          auto* module = createProbe(id, true);
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
      if (candidates && candidates[current.id])
        valid = applySetting(*candidates[current.id], line, current.type == HotType::Digital);
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
  if (!readHotLoadConfiguration(path, proposed, candidates, result)) return result;
  for (int i = 0; i < proposed.size(); ++i) {
    auto section = proposed.get(i);
    proposedIds[section.id] = true;
    auto* previous = _dmm->getModuleById(section.id);
    if (!previous) {
      if (section.type == HotType::StartupOnly) return HotLoadResult::RestartRequired;
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
      if (types[id] == HotType::Digital && (previousPins[id] & ~(uint64_t{3} << 32)))
        return HotLoadResult::RestartRequired;
    }
    if (!candidates[id]) continue;
    const uint64_t requested = hotLoadPins(candidates[id], types[id]);
    if (!requested) return HotLoadResult::InvalidConfiguration;
    if (claimed & requested) return HotLoadResult::ResourceConflict;
    claimed |= requested;
    for (uint8_t pin = 0; pin < DRONE_SYSTEM_PINS; ++pin) {
      if (!(requested & (uint64_t{1} << pin))) continue;
      const auto &entry = _ds->_pins[pin];
      if (entry.state == DRONE_SYSTEM_PIN_STATE_AVAILABLE &&
          (entry.capabilities & DRONE_SYSTEM_PIN_CAP_INPUT)) continue;
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
  if (_addressQueue.size()) return HotLoadResult::InvalidConfiguration;

  xSemaphoreTake(_ds->_moduleChangeMutex, portMAX_DELAY);
  // Allocate registry entries while the previous modules still own their pins.
  bool prepared = true;
  for (auto* module : candidates) {
    if (!module) continue;
    if (!_dmm->registerModule(module) || !_dlm->subscribe(module->id(), module, DRONE_LINK_PARAM_ALL)) {
      prepared = false;
      break;
    }
  }
  bool started = prepared;
  if (prepared) {
    for (unsigned id = 1; id <= MaxChannel; ++id)
      if (retired[id]) releaseHotLoadPins(retired[id], types[id]);
    for (unsigned id = 1; id <= MaxChannel; ++id) {
      auto* module = candidates[id];
      if (!module) continue;
      module->setup();
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
  if (type == HotType::Probe) return static_cast<ProbeModule*>(module)->requestedInputs();
  const auto pin = static_cast<DigitalModule*>(module)->trialPin();
  return pin == 32 || pin == 33 ? uint64_t{1} << pin : 0;
}

void DroneExecutionManager::releaseHotLoadPins(DroneModule* module, HotType type) {
  if (type == HotType::Probe) static_cast<ProbeModule*>(module)->releaseInputs();
  else static_cast<DigitalModule*>(module)->releaseHotLoadPin();
}

bool DroneExecutionManager::restoreHotLoadPins(DroneModule* module, HotType type, uint64_t pins) {
  if (!pins) return true;
  if (type == HotType::Probe) return static_cast<ProbeModule*>(module)->restoreInputs(pins);
  return static_cast<DigitalModule*>(module)->restoreHotLoadPin(pins & (uint64_t{1} << 32) ? 32 : 33);
}
