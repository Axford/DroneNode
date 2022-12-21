#include "Arduino.h"
#include "WiFiManager.h"
#include "WiFi.h"
#include "esp_wifi.h"
//#include "SPIFFS.h"

//#include "esp_int_wdt.h"

/*

Process

* Start in AP + STA mode
* Start a background scan for networks
* Once scan complete, look for matches to configured networks (priority order)
* If match found, try to connect
* If failed, try other matches
* when run out of matches and not connected, repeat background scan every xx seconds

* Leave in AP + STA mode, to allow backup configuration process and local connectivity

*/

WiFiManager::WiFiManager() {
  _taskCore = 1;
  _taskPriority = 1;
  _connectionCounter = 0;
  _APMode = true;
  _scanActive = false;
  _attemptingConnection = false;
  _enabled = true;
}


void WiFiManager::loadConfiguration(fs::FS &fs) {
  boolean addDefault = true;

  if (fs.exists(F("/wifi.json"))) {
    File file = fs.open(F("/wifi.json"), FILE_READ);

    DynamicJsonDocument doc(1024);

    DeserializationError error = deserializeJson(doc, file);
    if (error) {
      Log.errorln(F("[WIFI] Failed to read wifi.json file, using default configuration"));
    } else {
      JsonObject obj = doc.as<JsonObject>();

      // read networks
      WiFiNetworkCredentials cred;
      if (obj.containsKey(F("networks"))) {
        Log.noticeln("[WIFI] Parsing networks...");
        JsonArray array = obj[F("networks")].as<JsonArray>();
        for(JsonVariant v : array) {
          cred.ssid = v[F("ssid")] | "";
          cred.password = v[F("password")] | "";
          if (cred.ssid != "") {
            Log.noticeln("[WIFI] Adding %s, %s", cred.ssid.c_str(), cred.password.c_str());
            _networks.add(cred);
            addDefault = false;
          }
        }
      }
    }

    // Close the file (Curiously, File's destructor doesn't close the file)
    file.close();
  }

  if (addDefault ){
    Log.errorln(F("[WIFI] wifi.json file does not exist, adding defaults"));
    WiFiNetworkCredentials cred;
    // add default credentials
    cred.ssid = "Badger";
    cred.password = "LouisVuitton";
    //cred.ssid = "baladins";
    //cred.password = "lulutte2";
    _networks.add(cred);
  }
}

boolean WiFiManager::isEnabled() {
  return _enabled;
}

void WiFiManager::enable() {
  if (_enabled) return;

  //WiFi.mode(WIFI_AP_STA);
  WiFi.mode(WIFI_STA);
  vTaskDelay(2);

  // reduce wifi tx power to avoid brownout on V4 motherboards
  WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  String hostname = "Drone";
  for (uint8_t i=3; i<6; i++) {
    hostname += String(mac[i], HEX);
  }

  //WiFi.softAP(hostname.c_str());
  vTaskDelay(2);

  _scanActive = false;
  _attemptingConnection = false;

  _enabled = true;
}

void WiFiManager::disable() {
  if (!_enabled) return;

  _enabled = false;
  WiFi.disconnect();
  vTaskDelay(2);
  WiFi.mode(WIFI_OFF);
  vTaskDelay(2);
}


//const char * ssid, const char * pwd
void WiFiManager::start() {

  // speculative attempt to join first network in network list
  WiFi.begin(_networks[0].ssid.c_str(), _networks[0].password.c_str());

  //WiFi.disconnect();

/*
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(dmm->hostname().c_str());

  vTaskDelay(2);

  // speculative... on the off chance we have valid stored credentials
  WiFi.begin();
  */


  Log.noticeln("[WIFI] Starting connection task");

  WiFi.setSleep(false);

  xTaskCreatePinnedToCore(
    this->keepWiFiAlive,        //Function to implement the task
    "keepWiFiAlive",            //Name of the task
    5000,                   //Stack size in words
    this,                   //Task input parameter
    _taskPriority,           //Priority of the task
    &_Task1,                 //Task handle.
    _taskCore);              //Core where the task should run

  //esp_err_tesp_task_wdt_delete(&_Task1);
}

void WiFiManager::keepWiFiAlive(void *pvParameters){
  WiFiManager *l_pThis = (WiFiManager *) pvParameters;

  // pause a little while to allow the speculative start to have a chance of connecting
  vTaskDelay(5000 / portTICK_PERIOD_MS);

  for(;;){

    if (l_pThis->_enabled) {
      if(WiFi.status() == WL_CONNECTED){
        Log.noticeln("[WIFI] Connected: %p", WiFi.localIP());
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        continue;
      } else if (!l_pThis->_scanActive && !l_pThis->_attemptingConnection) {
        WiFi.disconnect();

        vTaskDelay(1000 / portTICK_PERIOD_MS);

        if (l_pThis->_enabled) {
          Log.noticeln("[WIFI] Not connected, scanning...");
          WiFi.scanNetworks(true, false, false, 300, 0);
          //bool async = false, bool show_hidden = false, bool passive = false, uint32_t max_ms_per_chan = 300, uint8_t channel = 0
          l_pThis->_scanActive = true;
          l_pThis->_attemptingConnection = false;

          vTaskDelay(1);
        }
      }

      if (l_pThis->_scanActive) {
        int16_t res = WiFi.scanComplete();
        if (res == WIFI_SCAN_RUNNING) {

        } else if (res == WIFI_SCAN_FAILED) {
          Log.noticeln("[WIFI] Scan failed");
          l_pThis->_scanActive = false;
          // scan failed... so wait a while before retry
          vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
          // ?
        } else if (res >=0 ) {
          Log.noticeln("[WIFI] Found %u networks", res);
          l_pThis->_scanActive = false;

          // see if any network matches one we have credentials for
          if (l_pThis->_enabled) {
            for (uint8_t i=0; i<l_pThis->_networks.size(); i++) {
              for (uint8_t j=0; j<res; j++) {
                Log.noticeln("[WIFI] Checking: %s = %s", l_pThis->_networks[i].ssid, WiFi.SSID(j));
                if (l_pThis->_networks[i].ssid.equals( WiFi.SSID(j) )) {
                  WiFi.disconnect();
                  vTaskDelay(1);
                  Log.noticeln("[WIFI] Match: %s", WiFi.SSID(j));
                  Log.noticeln("[WIFI] Connecting with password: %s", l_pThis->_networks[i].password.c_str());
                  WiFi.begin(l_pThis->_networks[i].ssid.c_str(), l_pThis->_networks[i].password.c_str());
                  l_pThis->_attemptingConnection = true;
                  vTaskDelay(1);
                }
              }
            }
          }

        }


      } else if (l_pThis->_attemptingConnection) {
        /*
        if (l_pThis->_connectionCounter > 0) {
          Serial.println("[WIFI] Off");
          WiFi.mode(WIFI_OFF);
        	vTaskDelay(1);
        }
        */

        //Serial.print("[WIFI] Connecting... SSID: ");
        //Serial.print(l_pThis->_ssid);
        //Serial.print(", pw: ");
        //Serial.println(l_pThis->_pwd);

        //WiFi.mode(WIFI_STA);
        //esp_wifi_set_ps (WIFI_PS_NONE);
        //Esp_wifi_set_ps (WIFI_PS_NONE);
        vTaskDelay(1);
        //WiFi.begin(l_pThis->_ssid, l_pThis->_pwd);

        unsigned long startAttemptTime = millis();

        // Keep looping while we're not connected and haven't reached the timeout
        Log.noticeln("[WIFI] waiting for connection");
        while (WiFi.status() != WL_CONNECTED &&
        millis() < WIFI_TIMEOUT_MS + startAttemptTime && l_pThis->_enabled){
          vTaskDelay(10);
        }

        if (WiFi.status() != WL_CONNECTED) {
          // abandon this attempt and trigger a rescan
          l_pThis->_attemptingConnection = false;
        }

        // When we couldn't make a WiFi connection (or the timeout expired)
        // sleep for a while and then retry.
        /*
        if(WiFi.status() != WL_CONNECTED){
          Serial.println("[WIFI] FAILED");
          vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
          continue;
        }*/

        l_pThis->_connectionCounter++;
      } else {
        // no connection in progress, sleep a while
        vTaskDelay(10);
      }
    } else {
      // currently disabled, so sleep a while
      Log.noticeln("~");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}
