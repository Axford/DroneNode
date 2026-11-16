#include <Arduino.h>
#include "ConfigTrialRuntime.h"
#include "BootRecovery.h"
#include "SPIFFS.h"
#include <functional>
#include <new>
#include "droneModules/DigitalModule.h"
#include "pinConfig.h"

#include "DroneModuleManager.h"
#include "DroneLinkManager.h"
#include "DroneModule.h"
#include "strings.h"
#include <ArduinoOTA.h>

DroneModuleManager::DroneModuleManager(DroneLinkManager* dlm, fs::FS &fs):
  _lastWatchdogCheck(0),
  _modules(IvanLinkedList::LinkedList<DroneModule*>()),
  _dlm(dlm),
  _fs(fs)
{
  _hostname = "set_me";
  _buildCommit = GIT_COMMIT;
  _lastDiscoveryIndex = 0;
  _node = 1;
  _doDiscovery = true;
  _lastDiscovery = 0;
  _sleep = 0;
};

// called by DroneModule constructor to self register on instantiation
bool DroneModuleManager::registerModule(DroneModule *m) {
  return _modules.add(m);
}

void DroneModuleManager::unregisterModule(DroneModule* module) {
  for (int i = 0; i < _modules.size(); ++i) {
    if (_modules.get(i) == module) {
      _modules.remove(i);
      _lastDiscoveryIndex = 0;
      return;
    }
  }
}

DroneModule* DroneModuleManager::getModuleByIndex(uint8_t index) {
  if (index >= _modules.size()) return NULL;
  return _modules.get(index);
}

DroneModule* DroneModuleManager::getModuleById(uint8_t id) {
  DroneModule* m;
  for(int i = 0; i < _modules.size(); i++) {
    m = _modules.get(i);
    if (m->id() == id) return m;
  }
  return NULL;
}

DroneModule* DroneModuleManager::getModuleByName(char * name) {
  DroneModule* m;
  for(int i = 0; i < _modules.size(); i++) {
    m = _modules.get(i);
    if (strcmp(m->getName(), name)==0) return m;
  }
  return NULL;
}

void DroneModuleManager::node(uint8_t id) {
  _node = id;
}

uint8_t DroneModuleManager::node() {
    return _node;
}

void DroneModuleManager::hostname(const char * name) {
  _hostname = name;
  WiFi.softAP(name);
  ArduinoOTA.setHostname(name);
}

String DroneModuleManager::hostname() {
  return _hostname;
}

boolean DroneModuleManager::discovery() { // get discovery state
  return _doDiscovery;
}

void DroneModuleManager::discovery(boolean v) { // set discovery state
  _doDiscovery = v;
}

void DroneModuleManager::setSleep(uint32_t sleep) {
  _sleep = sleep;
}


String DroneModuleManager::buildCommit() {
  return _buildCommit;
}


uint8_t DroneModuleManager::moduleCount() {
  return _modules.size();
}

void DroneModuleManager::onOTAProgress(float progress) {
  DroneModule* m;
  for(int i = 0; i < _modules.size(); i++) {
    m = _modules.get(i);
    m->onOTAProgress(progress);
  }
}

void DroneModuleManager::stopDigitalTrial() {
  auto* previous = _digitalTrial;
  _digitalTrial = nullptr;
  if (previous) previous->doShutdown();
  delete previous;
}

void DroneModuleManager::printDigitalTrial(Print &output) {
  if (!_digitalTrial) {
    output.println("trial state=off");
    return;
  }

  output.printf("trial state=running pin=%u level=%u\n",
                _digitalTrial->trialPin(), _digitalTrial->trialHigh());
}

void DroneModuleManager::runDigitalTrialCommand(const char* command, Print &output, DroneSystem* system) {
  if (strcmp(command, "trial") == 0) {
    printDigitalTrial(output);
    return;
  }
  if (strcmp(command, "trial off") == 0) {
    stopDigitalTrial();
    printDigitalTrial(output);
    return;
  }

  uint8_t pin;
  if (strcmp(command, "trial 32") == 0) {
    pin = PIN_OUT0_0;
  } else if (strcmp(command, "trial 33") == 0) {
    pin = PIN_OUT0_1;
  } else {
    output.println("trial error=invalid-command");
    printDigitalTrial(output);
    return;
  }

  if (!_digitalTrial || _digitalTrial->trialPin() != pin) {
    // Keep the previous input running until the candidate owns a pin and has sampled it.
    auto* candidate = new (std::nothrow) DigitalModule(0, system, DroneModule::Registration::Detached);
    const auto result = candidate ? candidate->startTrial(pin) : DigitalModule::TrialStart::OutOfMemory;
    if (result == DigitalModule::TrialStart::Started) {
      stopDigitalTrial();
      _digitalTrial = candidate;
    } else {
      delete candidate;
      output.println(result == DigitalModule::TrialStart::PinUnavailable ?
                     "trial error=pin-unavailable" : "trial error=out-of-memory");
    }
  }
  printDigitalTrial(output);
}

void DroneModuleManager::shutdown() {
  stopDigitalTrial();
  DroneModule* m;
  for(int i = 0; i < _modules.size(); i++) {
    m = _modules.get(i);
    m->doShutdown();
  }
}

void DroneModuleManager::restart() {
  bootRecovery::requestRestart();
  shutdown();
  ESP.restart();
}

void DroneModuleManager::updateStarting() {
  stopDigitalTrial();
  // disable discovery
  discovery(false);

  // shutdown non-essential modules
  DroneModule* m;
  for(int i = 1; i < _modules.size(); i++) {
    m = _modules.get(i);
    if (!m->getInterfaceState()) {
      m->doShutdown();
    } else {
      // also shutdown all non-UDP interfaces
      if (m->getInterfaceType() != DRONE_MESH_INTERFACE_TYPE_UDP) {
        m->doShutdown();
      }
    }
  }
}

void DroneModuleManager::setupModules() {
  DroneModule* m;
  for(int i = 0; i < _modules.size(); i++) {
    m = _modules.get(i);
    Log.noticeln(F("[DMM.sM] Setup: %s"), m->getName());
    char type[17] = {};
    const auto* entry = m->getParamEntryById(DRONE_MODULE_PARAM_TYPE);
    if (entry) memcpy(type, entry->data.c, 16);
    bootRecovery::enter(recovery::Phase::ModuleSetup, m->id(), type);
    m->setup();
    if (configTrial::running() && m->isEnabled()) {
      const auto* error = m->getParamEntryById(DRONE_MODULE_PARAM_ERROR);
      if (error && error->data.uint8[0]) configTrial::fail();
    }
    yield();
  }
}

void DroneModuleManager::loopModules() {
  if (_digitalTrial && _digitalTrial->readyToLoop()) _digitalTrial->loop();

  DroneModule* m;
  for(int i = 0; i < _modules.size(); i++) {
    m = _modules.get(i);
    //Log.noticeln(m->getName());
    unsigned long start= millis();
    boolean processed = false;

    // see if update needed
    processed = m->updateIfNeeded();

    // and check for main loop
    if (m->readyToLoop()) {
      //Serial.println(" Y");
      //Log.noticeln(F("[DMM.lM] %s"), m->getName());
      m->loop();
      processed = true;

    }

    // update loopDuration
    if (processed) {
      long duration = millis()-start;
      m->loopDuration = (m->loopDuration*15 + duration)/16;
    }

    yield();
  }


  // discovery broadcast?
  if (_doDiscovery) {
    unsigned long loopTime = millis();
    if (_modules.size() > 0) {
      if (loopTime > _lastDiscovery + DRONE_MODULE_MANAGER_DISCOVERY_INTERVAL) {
        m = _modules.get(_lastDiscoveryIndex);

        //Log.noticeln(F("[DMM.lM] disco: %s"), m->getName());

        if (m->doDiscovery()) {
          m->restartDiscovery();

          _lastDiscoveryIndex++;
          if (_lastDiscoveryIndex >= _modules.size()) {
            _lastDiscoveryIndex = 0;
          }
        }

        _lastDiscovery = loopTime;
      }
    }
  }

  // sleep a while?   ... only if wifi disabled
  if (_sleep > 0 && !_dlm->isWiFiEnabled()) {
    delay(1);  // in case stuff needs to finish?
    esp_sleep_enable_timer_wakeup(_sleep * 1000);  // in uS
    esp_light_sleep_start();
    yield();
  }
}

void DroneModuleManager::watchdog() {
  unsigned long loopTime = millis();
  if (loopTime > _lastWatchdogCheck + DRONE_MODULE_MANAGER_WATCHDOG_INTERVAL) {
    _lastWatchdogCheck = loopTime;

    DroneModule* m;
    for(int i = 0; i < _modules.size(); i++) {
      m = _modules.get(i);
      if (!m->isAlive()) {
        // seems dead... attempt a reset
        m->reset();
      }
    }
  }
}

void DroneModuleManager::serveModuleInfo(AsyncWebServerRequest *request) {

  AsyncResponseStream *response = request->beginResponseStream("application/json");
  response->addHeader("Server","ESP Async Web Server");
  response->print("[");
  DroneModule* m;
  for(uint8_t i = 0; i < _modules.size(); i++){
    m = _modules.get(i);
    if (i > 0) response->printf(",");
    response->print("{");
    response->printf("\"id\":%u,", m->id());

    m->respondWithInfo(response);

    // now print all params
    /*
    DEM_COMMAND c;
    for (uint8_t j=0; j<ns->commands->size(); j++) {
      c = ns->commands->get(j);
      response->printf("   %u: %s - %u\n", j, c.str, (c.handler != NULL ? 1 : 0));
    }
    */
    response->print("}\n");
  }

  response->print("]");

  //send the response last
  request->send(response);
}
