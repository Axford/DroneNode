#include "UDPTelemetryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "WiFi.h"

UDPTelemetryModule::UDPTelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs )
 {
   setTypeName(FPSTR(UDP_TELEMETRY_STR_UDP_TELEMETRY));
   _receivedSize = 0;
   _started = false;

   _packetsReceived = 0;
   _packetsRejected = 0;
   _packetsSent = 0;
   _packetsTimer = 0;

   // pubs
   initParams(UDP_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[UDP_PARAM_PORT_E];
   param->param = UDP_PARAM_PORT;
   setParamName(FPSTR(STRING_PORT), param);
   _params[UDP_PARAM_PORT_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[UDP_PARAM_PORT_E].data.uint32[0] = UDP_TELEMETRY_PORT;

   param = &_params[UDP_PARAM_BROADCAST_E];
   param->param = UDP_PARAM_BROADCAST;
   setParamName(FPSTR(STRING_BROADCAST), param);
   _params[UDP_PARAM_BROADCAST_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   _params[UDP_PARAM_BROADCAST_E].data.uint8[0] = 255;
   _params[UDP_PARAM_BROADCAST_E].data.uint8[1] = 255;
   _params[UDP_PARAM_BROADCAST_E].data.uint8[2] = 255;
   _params[UDP_PARAM_BROADCAST_E].data.uint8[3] = 255;

   param = &_params[UDP_TELEMETRY_PARAM_PACKETS_E];
   param->param = UDP_TELEMETRY_PARAM_PACKETS;
   setParamName(FPSTR(STRING_PACKETS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 12);
   param->data.uint32[0] = 0;
   param->data.uint32[1] = 0;
   param->data.uint32[2] = 0;

   param = &_params[UDP_TELEMETRY_PARAM_SPEED_E];
   param->param = UDP_TELEMETRY_PARAM_SPEED;
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[0] = 0;
   param->data.f[1] = 0;
   param->data.f[2] = 0;
}

DEM_NAMESPACE* UDPTelemetryModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(UDP_TELEMETRY_STR_UDP_TELEMETRY,0,true);
}

void UDPTelemetryModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_PORT, DRONE_LINK_MSG_TYPE_UINT32_T, ph);
  dem->registerCommand(ns, STRING_BROADCAST, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
}


void UDPTelemetryModule::handleLinkMessage(DroneLinkMsg *msg) {
  DroneModule::handleLinkMessage(msg);

  if (!_enabled || !_setupDone || !_started) return;

  // check to see if this is the same as the last message we received!
  // if so, we're getting stuck in a loop and the message should be ignored
  if (_receivedMsg.sameSignature(msg)) return;

  boolean wifiConnected = (WiFi.status() == WL_CONNECTED) || (WiFi.softAPIP()[0] > 0);
  if (!wifiConnected) return;

  IPAddress broadcastIp(
    _params[UDP_PARAM_BROADCAST_E].data.uint8[0],
    _params[UDP_PARAM_BROADCAST_E].data.uint8[1],
    _params[UDP_PARAM_BROADCAST_E].data.uint8[2],
    _params[UDP_PARAM_BROADCAST_E].data.uint8[3]
  );

  //Log.notice("UDP -> ");
  //msg->print();

  // only send messages that originate on this node
  boolean sendPacket = (msg->source() == _dlm->node());
  if (!sendPacket) {
    // OR!
    // on a different interface
    sendPacket = _dlm->getSourceInterface(msg->source()) != _id;

    // OR!!
    // that are queries
    if (!sendPacket) {
      sendPacket = msg->type() > DRONE_LINK_MSG_TYPE_CHAR;
    }
  }

  if (sendPacket) {
    _udp.beginPacket(broadcastIp, _params[UDP_PARAM_PORT_E].data.uint32[0]);
    _udp.write((uint8_t*)&msg->_msg, msg->length() + sizeof(DRONE_LINK_ADDR)+1);
    _udp.endPacket();

    _packetsSent++;
  }
}

void UDPTelemetryModule::setup() {
  DroneModule::setup();

  //setup deferred
}

void UDPTelemetryModule::loop() {
  DroneModule::loop();

  // deferred initialisation to allow for lack of wifi at start
  if (!_started && WiFi.status() == WL_CONNECTED) {
    Serial.println("[UDP.l] .begin");
    _udp.begin(_params[UDP_PARAM_PORT_E].data.uint32[0]);
    _started = true;
  }

  if (_started) {
    int packetSize = _udp.parsePacket();
    if (packetSize > 0 && packetSize <= sizeof(DRONE_LINK_MSG)) {
      //Log.noticeln("UDP <- ");
      int len = _udp.read((uint8_t*)&_receivedMsg._msg, sizeof(DRONE_LINK_MSG));
      if (len >= sizeof(DRONE_LINK_ADDR) + 2) {
        //_receivedMsg.print();
        _dlm->publishPeer(_receivedMsg, 0, _id);
        _packetsReceived++;
      } else if (packetSize > 0) {
        // error - packet size mismatch
        Log.errorln(F("UDPT: Packet size mismatch"));
        _packetsRejected++;
      }
    } else if (packetSize > 0) {
      Log.noticeln("UDP Rec bytes: %d", packetSize);
    }
  }

  // update and publish packet counters
  if (millis() > _packetsTimer + 5000) {

    uint32_t delta[3];
    delta[0] = _packetsSent - _params[UDP_TELEMETRY_PARAM_PACKETS_E].data.uint32[0];
    delta[1] = _packetsReceived - _params[UDP_TELEMETRY_PARAM_PACKETS_E].data.uint32[1];
    delta[2] = _packetsRejected - _params[UDP_TELEMETRY_PARAM_PACKETS_E].data.uint32[2];


    _params[UDP_TELEMETRY_PARAM_PACKETS_E].data.uint32[0] = _packetsSent;
    _params[UDP_TELEMETRY_PARAM_PACKETS_E].data.uint32[1] = _packetsReceived;
    _params[UDP_TELEMETRY_PARAM_PACKETS_E].data.uint32[2] = _packetsRejected;

    publishParamEntry(&_params[UDP_TELEMETRY_PARAM_PACKETS_E]);

    float dur = (millis() - _packetsTimer)/1000.0;
    for (uint8_t i=0; i<3; i++) {
      _params[UDP_TELEMETRY_PARAM_SPEED_E].data.f[i] = delta[i] / dur;
    }
    publishParamEntry(&_params[UDP_TELEMETRY_PARAM_SPEED_E]);

    _packetsTimer = millis();
  }
}
