#include <Arduino.h>
#include "SPIFFS.h"

#include "DroneModuleManager.h"
#include "DroneLinkManager.h"
#include "DroneModule.h"

// drone modules
#include "droneModules/ManagementModule.h"
#include "droneModules/TimerModule.h"
#include "droneModules/ServoModule.h"
#include "droneModules/TelemetryModule.h"
#include "droneModules/UDPTelemetryModule.h"
#include "droneModules/BME280Module.h"
#include "droneModules/INA219Module.h"
#include "droneModules/HMC5883LModule.h"
#include "droneModules/NMEAModule.h"
#include "droneModules/MPU6050Module.h"
#include "droneModules/MotorModule.h"
#include "droneModules/TankSteerModule.h"
#include "droneModules/BasicNavModule.h"
#include "droneModules/WaypointNavModule.h"
#include "droneModules/TurnRateModule.h"
#include "droneModules/RFM69TelemetryModule.h"
#include "droneModules/JoystickModule.h"
#include "droneModules/OLEDModule.h"
#include "droneModules/ControllerModule.h"
#include "droneModules/NunchuckJoystickModule.h"
#include "droneModules/NeopixelModule.h"


void DroneModuleManager::registerModule(DroneModule *m) {
  _modules.add(m);
}

DroneModule* DroneModuleManager::getModuleById(uint8_t id) {
  DroneModule* m;
  for(int i = 0; i < _modules.size(); i++) {
    m = _modules.get(i);
    if (m->id() == id) return m;
  }
  return NULL;
}

uint8_t DroneModuleManager::node() {
    return _node;
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

String DroneModuleManager::buildTimestamp() {
  return _buildTimestamp;
}

void DroneModuleManager::loadConfiguration() {
  Log.noticeln(F("[] Loading config..."));
  if (SPIFFS.exists(F("/config.json"))) {
    File file = SPIFFS.open(F("/config.json"), FILE_READ);

    // Allocate a temporary JsonDocument
    // Don't forget to change the capacity to match your requirements.
    // Use arduinojson.org/v6/assistant to compute the capacity.
    DynamicJsonDocument doc(6144);

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    if (error) {
      Log.errorln(F("[] Failed to read config file, using default configuration"));
    } else {

      JsonObject obj = doc.as<JsonObject>();

      // get node id
      _node = obj[F("node")] | _node;
      _dlm->node(_node);
      Log.noticeln(F("node=%d"), _node);

      // get hostname
      _hostname = obj[F("hostname")] | _hostname;
      Log.noticeln(F("hostname=%s"), _hostname.c_str());

      // discovery
      _doDiscovery = obj[F("discovery")] | _doDiscovery;

      // Parse and load modules
      // extract the values
      Log.noticeln(F("[] Parsing modules..."));

      if (obj.containsKey(F("modules"))) {
        loadModulesFromJSON(doc[F("modules")].as<JsonArray>());
      } else {
        Log.errorln(F("[] No modules found"));
      }
    }

    // Close the file (Curiously, File's destructor doesn't close the file)
    file.close();
  } else {
    Log.errorln(F("[] Config file does not exist"));
  }
}

void DroneModuleManager::loadModulesFromJSON(const JsonArray &array) {
  for(JsonVariant v : array) {
      if (v.is<JsonObject>()) {
        JsonObject modObj = v.as<JsonObject>();

        if (modObj.containsKey(DRONE_STR_TYPE)) {
          // decode module type typeName
          String typeName = modObj[DRONE_STR_TYPE].as<String>();

          // decode id
          uint8_t id = modObj[F("id")] | 255;

          Log.noticeln(F("Instantiating module: %s: %d"),typeName.c_str(), id);


          // instantiate appropriate module
          DroneModule *newMod = NULL;
          if (typeName.equals(MANAGEMENT_STR_MANAGEMENT)) {
            newMod = new ManagementModule(id, this, _dlm);
          } else if (typeName.equals(TELEMETRY_STR_TELEMETRY)) {
            newMod = new TelemetryModule(id, this, _dlm);
          } else if (typeName.equals(PSTR("UDPTelemetry"))) {
            newMod = new UDPTelemetryModule(id, this, _dlm);
          } else if (typeName.equals(SERVO_STR_SERVO)) {
            newMod = new ServoModule(id, this, _dlm);
          } else if (typeName.equals(PSTR("Timer"))) {
            newMod = new TimerModule(id, this, _dlm);
          } else if (typeName.equals(PSTR("WindSpeedDir"))) {
            //newMod = new WindSpeedDirModule(id, this, _dlm);
          } else if (typeName.equals(BME280_STR_BME280)) {
            newMod = new BME280Module(id, this, _dlm);
          } else if (typeName.equals(INA219_STR_INA219)) {
            newMod = new INA219Module(id, this, _dlm);
          } else if (typeName.equals(HMC5883L_STR_HMC5883L)) {
            newMod = new HMC5883LModule(id, this, _dlm);
          } else if (typeName.equals(NMEA_STR_NMEA)) {
            newMod = new NMEAModule(id, this, _dlm);
          } else if (typeName.equals(MPU6050_STR_MPU6050)) {
            newMod = new MPU6050Module(id, this, _dlm);
          } else if (typeName.equals(MOTOR_STR_MOTOR)) {
            newMod = new MotorModule(id, this, _dlm);
          } else if (typeName.equals(TANK_STEER_STR_TANK_STEER)) {
            newMod = new TankSteerModule(id, this, _dlm);
          } else if (typeName.equals(BASIC_NAV_STR_BASIC_NAV)) {
            newMod = new BasicNavModule(id, this, _dlm);
          } else if (typeName.equals(WAYPOINT_NAV_STR_WAYPOINT_NAV)) {
            newMod = new WaypointNavModule(id, this, _dlm);
          } else if (typeName.equals(TURN_RATE_STR_TURN_RATE)) {
            newMod = new TurnRateModule(id, this, _dlm);
          } else if (typeName.equals(RFM69_TELEMETRY_STR_RFM69_TELEMETRY)) {
            newMod = new RFM69TelemetryModule(id, this, _dlm);
          } else if (typeName.equals(JOYSTICK_STR_JOYSTICK)) {
            newMod = new JoystickModule(id, this, _dlm);
          } else if (typeName.equals(OLED_STR_OLED)) {
            newMod = new OLEDModule(id, this, _dlm);
          } else if (typeName.equals(CONTROLLER_STR_CONTROLLER)) {
            newMod = new ControllerModule(id, this, _dlm);
          } else if (typeName.equals(NunJOYSTICK_STR_NunJOYSTICK)) {
            newMod = new NunchuckJoystick(id, this, _dlm);
          } else if (typeName.equals(NEOPIXEL_STR_NEOPIXEL)) {
            newMod = new NeopixelModule(id, this, _dlm);
          } else {
            Log.errorln(F("Unknown type"));
          }

          // load further configuration
          if (newMod) {
            Log.noticeln(F(" Loading configuration..."));
            newMod->loadConfiguration(modObj);
          }

        } else {
            Log.errorln(F("[]  Undefined module type"));
        }
      }
  }
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

void DroneModuleManager::setupModules() {
  DroneModule* m;
  for(int i = 0; i < _modules.size(); i++) {
    m = _modules.get(i);
    m->setup();
  }
}

void DroneModuleManager::loopModules() {
  DroneModule* m;
  for(int i = 0; i < _modules.size(); i++) {
    m = _modules.get(i);
    //Serial.print(m->getName());
    if (m->readyToLoop()) {
      //Serial.println(" Y");
      m->loop();
    } else {
      //Serial.println(" N");
    }
  }

  // discovery broadcast?
  if (_doDiscovery) {
    unsigned long loopTime = millis();
    if (_modules.size() > 0) {
      if (loopTime > _lastDiscovery + DRONE_MODULE_MANAGER_DISCOVERY_INTERVAL) {
        m = _modules.get(_lastDiscoveryIndex);

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
