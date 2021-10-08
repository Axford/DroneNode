#include "UDPTelemetryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "WiFi.h"

UDPTelemetryModule::UDPTelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm):
  DroneModule ( id, dmm, dlm )
 {
   setTypeName(FPSTR(UDP_TELEMETRY_STR_UDP_TELEMETRY));
   _receivedSize = 0;
   _port = UDP_TELEMETRY_PORT;
   _broadcast[0] = 192;
   _broadcast[1] = 168;
   _broadcast[2] = 4;
   _broadcast[3] = 255;
}

void UDPTelemetryModule::loadConfiguration(JsonObject &obj) {
  DroneModule::loadConfiguration(obj);

  _port = obj[PSTR("port")] | _port;

  if (obj.containsKey(F("broadcast"))) {
    JsonArray arr = obj[F("broadcast")].as<JsonArray>();
    if (arr.size() == 4) {
      for (uint8_t i=0; i<4; i++) {
        _broadcast[i] = arr[i];
      }
    }
  }
}

void UDPTelemetryModule::handleLinkMessage(DroneLinkMsg *msg) {
  DroneModule::handleLinkMessage(msg);

  if (!_enabled) return;

  // check to see if this is the same as the last message we received!
  // if so, we're getting stuck in a loop and the message should be ignored
  if (_receivedMsg.sameSignature(msg)) return;

  boolean wifiConnected = (WiFi.status() == WL_CONNECTED) || (WiFi.softAPIP()[0] > 0);
  if (!wifiConnected) return;

  // TODO - load from config
  IPAddress broadcastIp(_broadcast[0], _broadcast[1], _broadcast[2], _broadcast[3]);

  //Log.noticeln("UDP Broadcast: ");
  //msg->print();

  // only send messages that originate on this node
  if (msg->source() == _dlm->node()) {
    _udp.beginPacket(broadcastIp, _port);
    _udp.write((uint8_t*)&msg->_msg, msg->length() + sizeof(DRONE_LINK_ADDR));
    _udp.endPacket();
  }
}

void UDPTelemetryModule::setup() {
  _udp.begin(_port);
}

void UDPTelemetryModule::loop() {
  DroneModule::loop();

  int packetSize = _udp.parsePacket();
  if (packetSize > 0 && packetSize <= sizeof(DRONE_LINK_MSG)) {
    //Log.noticeln("UDP Received: ");
    int len = _udp.read((uint8_t*)&_receivedMsg._msg, sizeof(DRONE_LINK_MSG));
    if (len >= sizeof(DRONE_LINK_ADDR) + 1) {
      //_receivedMsg.print();
      _dlm->publish(_receivedMsg);
    } else if (packetSize > 0) {
      // error - packet size mismatch
      Log.errorln(F("UDPT: Packet size mismatch"));
    }
  } else if (packetSize > 0) {
    Log.noticeln("UDP Rec bytes: %d", packetSize);
  }
}
