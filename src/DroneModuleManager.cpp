#include <Arduino.h>
#include "SPIFFS.h"
#include <functional>

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
void DroneModuleManager::registerModule(DroneModule *m) {
  _modules.add(m);
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

void DroneModuleManager::shutdown() {
  DroneModule* m;
  for(int i = 0; i < _modules.size(); i++) {
    m = _modules.get(i);
    m->doShutdown();
  }
}

void DroneModuleManager::restart() {
  shutdown();
  ESP.restart();
}

void DroneModuleManager::updateStarting() {
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
    m->setup();
    yield();
  }
}

void DroneModuleManager::loopModules() {
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
