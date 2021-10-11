#include "ManagementModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../DroneModuleManager.h"
#include "../OTAManager.h"


ManagementModule::ManagementModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(MANAGEMENT_STR_MANAGEMENT));

   _loopInterval = 30000;  // 30 sec
   _lastRate = 0;

   initParams(MANAGEMENT_PARAM_ENTRIES);

   // defaults
   for (uint8_t i=0; i<_numParamEntries; i++) {
     _params[i].data.uint8[0] = 0;
   }

   // init param entries
   _params[MANAGEMENT_PARAM_HOSTNAME_E].param = MANAGEMENT_PARAM_HOSTNAME;
   _params[MANAGEMENT_PARAM_HOSTNAME_E].name = FPSTR(DRONE_STR_HOSTNAME);
   _params[MANAGEMENT_PARAM_HOSTNAME_E].nameLen = sizeof(DRONE_STR_HOSTNAME);
   _params[MANAGEMENT_PARAM_HOSTNAME_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, _dmm->hostname().length());
   _dmm->hostname().toCharArray(_params[MANAGEMENT_PARAM_HOSTNAME_E].data.c, 16);

   _params[MANAGEMENT_PARAM_BUILD_E].param = MANAGEMENT_PARAM_BUILD;
   _params[MANAGEMENT_PARAM_BUILD_E].name = FPSTR(DRONE_STR_BUILD);
   _params[MANAGEMENT_PARAM_BUILD_E].nameLen = sizeof(DRONE_STR_BUILD);
   _params[MANAGEMENT_PARAM_BUILD_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_CHAR, _dmm->buildTimestamp().length());
   _dmm->buildTimestamp().toCharArray(_params[MANAGEMENT_PARAM_BUILD_E].data.c, 16);

   _params[MANAGEMENT_PARAM_RESET_E].param = MANAGEMENT_PARAM_RESET;
   _params[MANAGEMENT_PARAM_RESET_E].name = FPSTR(DRONE_STR_RESET);
   _params[MANAGEMENT_PARAM_RESET_E].nameLen = sizeof(DRONE_STR_RESET);
   _params[MANAGEMENT_PARAM_RESET_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);

   _params[MANAGEMENT_PARAM_HEAP_E].param = MANAGEMENT_PARAM_HEAP;
   _params[MANAGEMENT_PARAM_HEAP_E].name = FPSTR(DRONE_STR_HEAP);
   _params[MANAGEMENT_PARAM_HEAP_E].nameLen = sizeof(DRONE_STR_HEAP);
   _params[MANAGEMENT_PARAM_HEAP_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[MANAGEMENT_PARAM_HEAP_E].data.uint32[0] = ESP.getFreeHeap();

   _params[MANAGEMENT_PARAM_IP_E].param = MANAGEMENT_PARAM_IP;
   _params[MANAGEMENT_PARAM_IP_E].name = FPSTR(DRONE_STR_IP);
   _params[MANAGEMENT_PARAM_IP_E].nameLen = sizeof(DRONE_STR_IP);
   _params[MANAGEMENT_PARAM_IP_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 4);
   _params[MANAGEMENT_PARAM_IP_E].data.uint8[0] = 0;

   _params[MANAGEMENT_PARAM_UPTIME_E].param = MANAGEMENT_PARAM_UPTIME;
   _params[MANAGEMENT_PARAM_UPTIME_E].name = FPSTR(DRONE_STR_UPTIME);
   _params[MANAGEMENT_PARAM_UPTIME_E].nameLen = sizeof(DRONE_STR_UPTIME);
   _params[MANAGEMENT_PARAM_UPTIME_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[MANAGEMENT_PARAM_UPTIME_E].data.uint32[0] = 0;

   _params[MANAGEMENT_PARAM_PUBLISHRATE_E].param = MANAGEMENT_PARAM_PUBLISHRATE;
   _params[MANAGEMENT_PARAM_PUBLISHRATE_E].name = FPSTR(DRONE_STR_PUBLISHRATE);
   _params[MANAGEMENT_PARAM_PUBLISHRATE_E].nameLen = sizeof(DRONE_STR_PUBLISHRATE);
   _params[MANAGEMENT_PARAM_PUBLISHRATE_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 4);
   _params[MANAGEMENT_PARAM_PUBLISHRATE_E].data.f[0] = 0;

   _params[MANAGEMENT_PARAM_CHOKED_E].param = MANAGEMENT_PARAM_CHOKED;
   _params[MANAGEMENT_PARAM_CHOKED_E].name = FPSTR(DRONE_STR_CHOKED);
   _params[MANAGEMENT_PARAM_CHOKED_E].nameLen = sizeof(DRONE_STR_CHOKED);
   _params[MANAGEMENT_PARAM_CHOKED_E].paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[MANAGEMENT_PARAM_CHOKED_E].data.uint32[0] = 0;
}

void ManagementModule::onParamWrite(DRONE_PARAM_ENTRY *param) {
  DroneModule::onParamWrite(param);

  if (param->param == MANAGEMENT_PARAM_RESET) {
    // trigger local reset
    if (_params[MANAGEMENT_PARAM_RESET_E].data.uint8[0] > 0) {
      Serial.println("Restart requested");
      _dmm->restart();
    }
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

  // publish param entries
  //publishParamEntries();
}
