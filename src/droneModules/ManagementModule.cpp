#include "ManagementModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../DroneModuleManager.h"
#include "../DroneExecutionManager.h"
#include "../OTAManager.h"
#include "strings.h"
#include "OLEDTomThumbFont.h"

ManagementModule::ManagementModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(MANAGEMENT_STR_MANAGEMENT));

   _mgmtParams[DRONE_MODULE_PARAM_INTERVAL_E].data.uint32[0] = 30000;  // 30 sec
   _lastRate = 0;

   initParams(MANAGEMENT_PARAM_ENTRIES);

   // defaults
   for (uint8_t i=0; i<_numParamEntries; i++) {
     _params[i].data.uint8[0] = 0;
   }

   // init param entries
   _params[MANAGEMENT_PARAM_HOSTNAME_E].param = MANAGEMENT_PARAM_HOSTNAME;
   _params[MANAGEMENT_PARAM_HOSTNAME_E].name = FPSTR(STRING_HOSTNAME);
   _params[MANAGEMENT_PARAM_HOSTNAME_E].nameLen = sizeof(STRING_HOSTNAME);
   _params[MANAGEMENT_PARAM_HOSTNAME_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_CHAR, _dmm->hostname().length());
   _dmm->hostname().toCharArray(_params[MANAGEMENT_PARAM_HOSTNAME_E].data.c, 16);

   _params[MANAGEMENT_PARAM_BUILD_E].param = MANAGEMENT_PARAM_BUILD;
   _params[MANAGEMENT_PARAM_BUILD_E].name = FPSTR(STRING_BUILD);
   _params[MANAGEMENT_PARAM_BUILD_E].nameLen = sizeof(STRING_BUILD);
   _params[MANAGEMENT_PARAM_BUILD_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, _dmm->buildCommit().length());
   _dmm->buildCommit().toCharArray(_params[MANAGEMENT_PARAM_BUILD_E].data.c, 16);

   _params[MANAGEMENT_PARAM_RESET_E].param = MANAGEMENT_PARAM_RESET;
   _params[MANAGEMENT_PARAM_RESET_E].name = FPSTR(STRING_RESET);
   _params[MANAGEMENT_PARAM_RESET_E].nameLen = sizeof(STRING_RESET);
   _params[MANAGEMENT_PARAM_RESET_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);

   _params[MANAGEMENT_PARAM_HEAP_E].param = MANAGEMENT_PARAM_HEAP;
   _params[MANAGEMENT_PARAM_HEAP_E].name = FPSTR(STRING_HEAP);
   _params[MANAGEMENT_PARAM_HEAP_E].nameLen = sizeof(STRING_HEAP);
   _params[MANAGEMENT_PARAM_HEAP_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[MANAGEMENT_PARAM_HEAP_E].data.uint32[0] = ESP.getFreeHeap();

   _params[MANAGEMENT_PARAM_IP_E].param = MANAGEMENT_PARAM_IP;
   _params[MANAGEMENT_PARAM_IP_E].name = FPSTR(STRING_IP);
   _params[MANAGEMENT_PARAM_IP_E].nameLen = sizeof(STRING_IP);
   _params[MANAGEMENT_PARAM_IP_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 4);
   _params[MANAGEMENT_PARAM_IP_E].data.uint8[0] = 0;

   _params[MANAGEMENT_PARAM_UPTIME_E].param = MANAGEMENT_PARAM_UPTIME;
   _params[MANAGEMENT_PARAM_UPTIME_E].name = FPSTR(STRING_UPTIME);
   _params[MANAGEMENT_PARAM_UPTIME_E].nameLen = sizeof(STRING_UPTIME);
   _params[MANAGEMENT_PARAM_UPTIME_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[MANAGEMENT_PARAM_UPTIME_E].data.uint32[0] = 0;

   _params[MANAGEMENT_PARAM_PUBLISHRATE_E].param = MANAGEMENT_PARAM_PUBLISHRATE;
   _params[MANAGEMENT_PARAM_PUBLISHRATE_E].name = FPSTR(STRING_PUBLISHRATE);
   _params[MANAGEMENT_PARAM_PUBLISHRATE_E].nameLen = sizeof(STRING_PUBLISHRATE);
   _params[MANAGEMENT_PARAM_PUBLISHRATE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[MANAGEMENT_PARAM_PUBLISHRATE_E].data.f[0] = 0;

   _params[MANAGEMENT_PARAM_CHOKED_E].param = MANAGEMENT_PARAM_CHOKED;
   _params[MANAGEMENT_PARAM_CHOKED_E].name = FPSTR(STRING_CHOKED);
   _params[MANAGEMENT_PARAM_CHOKED_E].nameLen = sizeof(STRING_CHOKED);
   _params[MANAGEMENT_PARAM_CHOKED_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[MANAGEMENT_PARAM_CHOKED_E].data.uint32[0] = 0;

   _params[MANAGEMENT_PARAM_DISCOVERY_E].param = MANAGEMENT_PARAM_DISCOVERY;
   _params[MANAGEMENT_PARAM_DISCOVERY_E].name = FPSTR(STRING_DISCOVERY);
   _params[MANAGEMENT_PARAM_DISCOVERY_E].nameLen = sizeof(STRING_DISCOVERY);
   _params[MANAGEMENT_PARAM_DISCOVERY_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[MANAGEMENT_PARAM_DISCOVERY_E].data.uint8[0] = _dmm->discovery() ? 1 : 0;

   _params[MANAGEMENT_PARAM_MACRO_E].param = MANAGEMENT_PARAM_MACRO;
   _params[MANAGEMENT_PARAM_MACRO_E].name = FPSTR(STRING_MACRO);
   _params[MANAGEMENT_PARAM_MACRO_E].nameLen = sizeof(STRING_MACRO);
   _params[MANAGEMENT_PARAM_MACRO_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_CHAR, 1);
   _params[MANAGEMENT_PARAM_MACRO_E].data.c[0] = 0;

   _params[MANAGEMENT_PARAM_SLEEP_E].param = MANAGEMENT_PARAM_SLEEP;
   _params[MANAGEMENT_PARAM_SLEEP_E].name = FPSTR(STRING_SLEEP);
   _params[MANAGEMENT_PARAM_SLEEP_E].nameLen = sizeof(STRING_SLEEP);
   _params[MANAGEMENT_PARAM_SLEEP_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[MANAGEMENT_PARAM_SLEEP_E].data.uint32[0] = 0;

   _params[MANAGEMENT_PARAM_WIFI_E].param = MANAGEMENT_PARAM_WIFI;
   _params[MANAGEMENT_PARAM_WIFI_E].name = FPSTR(STRING_WIFI);
   _params[MANAGEMENT_PARAM_WIFI_E].nameLen = sizeof(STRING_WIFI);
   _params[MANAGEMENT_PARAM_WIFI_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[MANAGEMENT_PARAM_WIFI_E].data.uint8[0] = 1; // 1 = enabled

   _params[MANAGEMENT_PARAM_CPU_E].param = MANAGEMENT_PARAM_CPU;
   _params[MANAGEMENT_PARAM_CPU_E].name = FPSTR(STRING_CPU);
   _params[MANAGEMENT_PARAM_CPU_E].nameLen = sizeof(STRING_CPU);
   _params[MANAGEMENT_PARAM_CPU_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[MANAGEMENT_PARAM_CPU_E].data.uint8[0] = 240; // default high speed
}

DEM_NAMESPACE* ManagementModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(MANAGEMENT_STR_MANAGEMENT,0,true);
}

void ManagementModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_RESET, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_DISCOVERY, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_HOSTNAME, DRONE_LINK_MSG_TYPE_CHAR, ph);
  dem->registerCommand(ns, STRING_WIFI, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_SLEEP, DRONE_LINK_MSG_TYPE_UINT32_T, ph);
  dem->registerCommand(ns, STRING_CPU, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
}

void ManagementModule::onParamWrite(DRONE_PARAM_ENTRY *param) {
  DroneModule::onParamWrite(param);

  if (param->param == MANAGEMENT_PARAM_RESET) {
    // trigger local reset
    if (_params[MANAGEMENT_PARAM_RESET_E].data.uint8[0] > 0) {
      Log.noticeln("Restart requested");
      _dmm->restart();
    }
  }

  if (param->param == MANAGEMENT_PARAM_DISCOVERY) {
    // update discovery state
    Log.noticeln("Changing discovery mode to: %u", _params[MANAGEMENT_PARAM_DISCOVERY_E].data.uint8[0]);
    _dmm->discovery( _params[MANAGEMENT_PARAM_DISCOVERY_E].data.uint8[0] > 0 );
  }

  if (param->param == MANAGEMENT_PARAM_MACRO) {
    // attempt to execute the macro requested
    // null terminate to be safe
    uint8_t len = min(DRONE_LINK_MSG_MAX_PAYLOAD-1, (_params[MANAGEMENT_PARAM_MACRO_E].paramTypeLength & 0xF) + 1   );
    _params[MANAGEMENT_PARAM_MACRO_E].data.c[len] = 0;
    Log.noticeln(F("[MM.oPW] macro... "));
    _dem->runMacro(_params[MANAGEMENT_PARAM_MACRO_E].data.c, false);
  }

  if (param->param == MANAGEMENT_PARAM_HOSTNAME) {
    // ensure null terminated
    uint8_t len = (_params[MANAGEMENT_PARAM_HOSTNAME_E].paramTypeLength & 0xF) + 1;
    len = min(len, (uint8_t)15);
    _params[MANAGEMENT_PARAM_HOSTNAME_E].data.c[len] = 0;
    _dmm->hostname((const char*)&_params[MANAGEMENT_PARAM_HOSTNAME_E].data.c);
  }

  if (param->param == MANAGEMENT_PARAM_DISCOVERY) {
    // update discovery state
    Log.noticeln("[MM.oPW] Changing discovery mode to: %u", _params[MANAGEMENT_PARAM_DISCOVERY_E].data.uint8[0]);
    _dmm->discovery( _params[MANAGEMENT_PARAM_DISCOVERY_E].data.uint8[0] > 0 );
  }

  if (param->param == MANAGEMENT_PARAM_WIFI) {
    // update wifi state
    Log.noticeln("[MM.oPW] Changing wifi mode to: %u", _params[MANAGEMENT_PARAM_WIFI_E].data.uint8[0]);
    if (_params[MANAGEMENT_PARAM_WIFI_E].data.uint8[0] == 1) {
      _dlm->enableWiFi();
    } else {
      _dlm->disableWiFi();
    }
  }

  if (param->param == MANAGEMENT_PARAM_SLEEP) {
    // update DMM interval
    Log.noticeln("[MM.oPW] Changing DMM interval to: %u", _params[MANAGEMENT_PARAM_SLEEP_E].data.uint32[0]);
    _dmm->setSleep(_params[MANAGEMENT_PARAM_SLEEP_E].data.uint32[0]);
  }

  if (param->param == MANAGEMENT_PARAM_CPU) {
    // update DMM interval
    Log.noticeln("[MM.oPW] Changing CPU to: %u MHz", _params[MANAGEMENT_PARAM_CPU_E].data.uint8[0]);
    setCpuFrequencyMhz(_params[MANAGEMENT_PARAM_CPU_E].data.uint8[0]);
  }
}

void ManagementModule::setup() {
  DroneModule::setup();
}

void ManagementModule::loop() {
  DroneModule::loop();

  // update stats
  uint32_t temp32 = ESP.getFreeHeap();
  updateAndPublishParam(&_params[MANAGEMENT_PARAM_HEAP_E], (uint8_t*)&temp32, sizeof(temp32));

  temp32 = millis() / 1000;
  updateAndPublishParam(&_params[MANAGEMENT_PARAM_UPTIME_E], (uint8_t*)&temp32, sizeof(temp32));

  temp32 = _dlm->getChokes();
  updateAndPublishParam(&_params[MANAGEMENT_PARAM_CHOKED_E], (uint8_t*)&temp32, sizeof(temp32));

  // fetch IP address
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ipa = WiFi.localIP();
    temp32 = ipa[3]<<24 | ipa[2]<<16 | ipa[1]<<8 | ipa[0];
  } else {
    temp32 = 0;
  }
  updateAndPublishParam(&_params[MANAGEMENT_PARAM_IP_E], (uint8_t*)&temp32, sizeof(temp32));

  if (_lastLoop > _lastRate) {
    float tempf = _dlm->publishedMessages() / ((_lastLoop - _lastRate) / 1000.0f);
    updateAndPublishParam(&_params[MANAGEMENT_PARAM_PUBLISHRATE_E], (uint8_t*)&tempf, sizeof(tempf));
    _dlm->resetPublishedMessages();

    _lastRate = _lastLoop;
  }
}


uint8_t ManagementModule::diagnosticDisplays() {
  return 1;
}

void ManagementModule::drawDiagnosticDisplay(SSD1306Wire *display, uint8_t page) {
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  if (page == 0)
  {
    // page 0
    display->setFont(TomThumb4x6);
    display->drawString(0, 17+4, "IP");
    display->setFont(ArialMT_Plain_10);

    for (uint8_t i=0; i<4; i++) {
      display->drawString(32 + i*20, 17, String(_params[MANAGEMENT_PARAM_IP_E].data.uint8[i]));
    }

    // uptime
    display->setFont(TomThumb4x6);
    display->drawString(0, 34+4, "UPTIME");
    display->setFont(ArialMT_Plain_10);
    display->drawString(32, 34, String(millis()/1000) + "s");
  } else {
    // page 1
    //display->drawString(2, 17, "page 1");
  }
}
