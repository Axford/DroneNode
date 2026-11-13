#include "DroneSystem.h"
#include "droneModules/DigitalModule.h"
#include "droneModules/ManagementModule.h"
#include "CloudTelemetryConfig.h"
#include <mbedtls/sha256.h>
#include <math.h>
#include <new>

using configuration::HotLoadResult;

namespace {

constexpr size_t MaxFileSize = 8192;
constexpr uint8_t MaxChannel = 254;

bool applyDigitalSetting(DigitalModule &module, configuration::Line &line) {
  if (!strcmp(line.name, "publish")) {
    char* remaining = line.value;
    while (remaining) {
      char* name = strsep(&remaining, ",");
      auto* param = module.getParamEntryByName(name);
      if (!param) return false;
      param->publish = true;
    }
    return true;
  }

  auto* param = module.getParamEntryByName(line.name);
  if (!param) return false;
  const auto id = getDroneLinkMsgParam(param->paramPriority);
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
  DigitalModule* modules[MaxChannel + 1] = {};
  DigitalModule* retired[MaxChannel + 1] = {};
  bool proposedIds[MaxChannel + 1] = {};
  uint8_t previousPins[MaxChannel + 1] = {};
  ~Candidates() {
    for (auto* module : modules) delete module;
  }
};

} // namespace

bool DroneExecutionManager::readHotLoadConfiguration(
    const char* path, IvanLinkedList::LinkedList<AppliedSection>& sections,
    DigitalModule** candidates, HotLoadResult& result) {
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
    const char* raw = strsep(&remaining, "\n");
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
      current.digital = !strcmp(line.name, DIGITAL_STR_DIGITAL);
      mbedtls_sha256_starts_ret(&hash, 0);
      add(line.name);
      if (candidates && current.digital) {
        auto* module = createDigital(id, true);
        candidates[id] = module;
        if (!module || !module->hotLoadReady()) {
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
      if (candidates && current.digital) valid = applyDigitalSetting(*candidates[current.id], line);
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
      if (!section.digital) return HotLoadResult::RestartRequired;
      continue;
    }
    for (int j = 0; j < _appliedSections.size(); ++j) {
      auto active = _appliedSections.get(j);
      if (active.id != section.id) continue;
      if (!memcmp(active.digest, section.digest, sizeof(active.digest))) {
        delete candidates[section.id];
        candidates[section.id] = nullptr;
      } else if (active.digital && section.digital) {
        // Preserve consumer binding and publication semantics for this first lifecycle.
        if (_dlm->hasLocalConsumer(section.id, previous)) return HotLoadResult::InvalidConfiguration;
        retired[section.id] = static_cast<DigitalModule*>(previous);
      } else {
        return HotLoadResult::RestartRequired;
      }
      break;
    }
  }
  for (int i = 0; i < _appliedSections.size(); ++i) {
    auto section = _appliedSections.get(i);
    if (proposedIds[section.id]) continue;
    if (!section.digital) return HotLoadResult::RestartRequired;
    auto* module = _dmm->getModuleById(section.id);
    if (_dlm->hasLocalConsumer(section.id, module)) return HotLoadResult::InvalidConfiguration;
    retired[section.id] = static_cast<DigitalModule*>(module);
  }

  DigitalModule* previousOwner[2] = {};
  for (unsigned id = 1; id <= MaxChannel; ++id) {
    if (!retired[id]) continue;
    const uint8_t pin = retired[id]->ownedPin();
    previousPins[id] = pin;
    if (pin == 255) continue;
    if (pin != 32 && pin != 33) return HotLoadResult::RestartRequired;
    previousOwner[pin - 32] = retired[id];
  }
  bool claimed[2] = {};
  for (auto* module : candidates) {
    if (!module) continue;
    const uint8_t pin = module->trialPin();
    if (pin != 32 && pin != 33) return HotLoadResult::InvalidConfiguration;
    if (claimed[pin - 32] || !_ds->canClaimDigitalPin(pin, previousOwner[pin - 32]))
      return HotLoadResult::ResourceConflict;
    claimed[pin - 32] = true;
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
    for (auto* module : retired) if (module) module->releaseHotLoadPin();
    for (auto* module : candidates) {
      if (module && !module->startHotLoad()) {
        started = false;
        break;
      }
    }
  }
  if (!started) {
    for (auto* module : candidates) {
      if (!module) continue;
      _dmm->unregisterModule(module);
      _dlm->detachModule(module);
      module->releaseHotLoadPin();
    }
    result = prepared ? HotLoadResult::StartFailed : HotLoadResult::OutOfMemory;
    if (prepared) {
      for (unsigned id = 1; id <= MaxChannel; ++id) {
        if (retired[id] && previousPins[id] != 255 && !retired[id]->restoreHotLoadPin(previousPins[id])) {
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
