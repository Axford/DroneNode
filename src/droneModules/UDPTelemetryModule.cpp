#include "UDPTelemetryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "WiFi.h"

// @type UDPTelemetry

UDPTelemetryModule::UDPTelemetryModule(uint8_t id, DroneSystem* ds):
  NetworkInterfaceModule ( id, ds )
 {
   setTypeName(FPSTR(UDP_TELEMETRY_STR_UDP_TELEMETRY));
   _receivedSize = 0;
   _started = false;

   _packetsReceived = 0;
   _packetsRejected = 0;
   _packetsSent = 0;
   _packetsTimer = 0;

   _broadcastCapable = true;

   // pubs
   initParams(UDP_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[UDP_PARAM_PORT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, UDP_PARAM_PORT);
   setParamName(FPSTR(STRING_PORT), param);
   _params[UDP_PARAM_PORT_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   // @default port=8007
   _params[UDP_PARAM_PORT_E].data.uint32[0] = UDP_TELEMETRY_PORT;

   param = &_params[UDP_PARAM_BROADCAST_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, UDP_PARAM_BROADCAST);
   setParamName(FPSTR(STRING_BROADCAST), param);
   _params[UDP_PARAM_BROADCAST_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 4);
   // @default broadcast=255,255,255,255
   _params[UDP_PARAM_BROADCAST_E].data.uint8[0] = 255;
   _params[UDP_PARAM_BROADCAST_E].data.uint8[1] = 255;
   _params[UDP_PARAM_BROADCAST_E].data.uint8[2] = 255;
   _params[UDP_PARAM_BROADCAST_E].data.uint8[3] = 255;

   param = &_params[UDP_TELEMETRY_PARAM_PACKETS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, UDP_TELEMETRY_PARAM_PACKETS);
   setParamName(FPSTR(STRING_PACKETS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 12);
   param->data.uint32[0] = 0;
   param->data.uint32[1] = 0;
   param->data.uint32[2] = 0;

   param = &_params[UDP_TELEMETRY_PARAM_SPEED_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, UDP_TELEMETRY_PARAM_SPEED);
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[0] = 0;
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[UDP_TELEMETRY_PARAM_SERVER_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, UDP_TELEMETRY_PARAM_SERVER);
   setParamName(FPSTR(STRING_SERVER), param);
   _params[UDP_TELEMETRY_PARAM_SERVER_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 4);
   // @default server=0,0,0,0
   _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[0] = 0;
   _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[1] = 0;
   _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[2] = 0;
   _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[3] = 0;

   param = &_params[UDP_TELEMETRY_PARAM_URL_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, UDP_TELEMETRY_PARAM_URL);
   setParamName(FPSTR(STRING_URL), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_CHAR, 16);
   param->data.c[0] = 0;
}

/*
IPAddress server_ip;
WiFi.hostByName("www.google.com",server_ip);
Serial.println(server_ip);
*/


uint8_t UDPTelemetryModule::getInterfaceType() {
  // to be overridden
  return DRONE_MESH_INTERFACE_TYPE_UDP;
}


void UDPTelemetryModule::setup() {
  NetworkInterfaceModule::setup();

  // register network interface
  _dlm->registerInterface(this);

  // rest of setup is deferred until a WiFi connection is available (see loop)
}


void UDPTelemetryModule::loop() {
  NetworkInterfaceModule::loop();

  unsigned long loopTime = millis();

  // deferred initialisation to allow for lack of wifi at start
  if (!_started && WiFi.status() == WL_CONNECTED) {
    Log.noticeln("[UDP.l] .begin");
    // start UDP server
    _udp.begin(_params[UDP_PARAM_PORT_E].data.uint32[0]);
    _started = true;
  }

  // maintain interface state
  _interfaceState = (WiFi.status() == WL_CONNECTED);

  if (_started) {
    int packetSize = _udp.parsePacket();
    if (packetSize > 0 && packetSize <= DRONE_MESH_MSG_MAX_PACKET_SIZE) {


      int len = _udp.read(_rBuffer, DRONE_MESH_MSG_MAX_PACKET_SIZE);
      if (len >= sizeof(DRONE_MESH_MSG_HEADER) + 2) {
        // convert WiFi RSSI to receive metric
        long rssi = abs(constrain(WiFi.RSSI(), -100, 0));
        uint8_t metric = map(rssi, 0, 100, 1, 15);

        DRONE_LINK_TRANSPORT_ADDRESS ta;
        ta.ipAddress = _udp.remoteIP();

        receivePacket(_rBuffer, metric, ta);
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
  if (loopTime > _packetsTimer + 5000) {

    uint32_t delta[3];
    delta[0] = _packetsSent - _params[UDP_TELEMETRY_PARAM_PACKETS_E].data.uint32[0];
    delta[1] = _packetsReceived - _params[UDP_TELEMETRY_PARAM_PACKETS_E].data.uint32[1];
    delta[2] = _packetsRejected - _params[UDP_TELEMETRY_PARAM_PACKETS_E].data.uint32[2];


    _params[UDP_TELEMETRY_PARAM_PACKETS_E].data.uint32[0] = _packetsSent;
    _params[UDP_TELEMETRY_PARAM_PACKETS_E].data.uint32[1] = _packetsReceived;
    _params[UDP_TELEMETRY_PARAM_PACKETS_E].data.uint32[2] = _packetsRejected;

    publishParamEntry(&_params[UDP_TELEMETRY_PARAM_PACKETS_E]);

    float dur = (loopTime - _packetsTimer)/1000.0;
    for (uint8_t i=0; i<3; i++) {
      _params[UDP_TELEMETRY_PARAM_SPEED_E].data.f[i] = delta[i] / dur;
    }
    publishParamEntry(&_params[UDP_TELEMETRY_PARAM_SPEED_E]);

    // take opportunity to resolve server address...
    if (_interfaceState &&
        _params[UDP_TELEMETRY_PARAM_URL_E].data.c[0] != 0 &&
        _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[0] == 0
       ) {
        IPAddress serverAddress;
        WiFi.hostByName(_params[UDP_TELEMETRY_PARAM_URL_E].data.c, serverAddress);

        if (serverAddress[0] != 0) {
          // copy into server param
          
          _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[0] = serverAddress[0];
          _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[1] = serverAddress[1];
          _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[2] = serverAddress[2];
          _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[3] = serverAddress[3];

          publishParamEntry(&_params[UDP_TELEMETRY_PARAM_SERVER_E]);
        }
    }

    _packetsTimer = loopTime;
  }
}


boolean UDPTelemetryModule::sendPacket(uint8_t *buffer, DRONE_LINK_TRANSPORT_ADDRESS transportAddress) {
  if (!_enabled) return false;

  // can only send if WiFi connected
  boolean wifiConnected = (WiFi.status() == WL_CONNECTED) || (WiFi.softAPIP()[0] > 0);
  if (!wifiConnected) return false;

  IPAddress broadcastIp(
    _params[UDP_PARAM_BROADCAST_E].data.uint8[0],
    _params[UDP_PARAM_BROADCAST_E].data.uint8[1],
    _params[UDP_PARAM_BROADCAST_E].data.uint8[2],
    _params[UDP_PARAM_BROADCAST_E].data.uint8[3]
  );

  IPAddress serverIp(
    _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[0],
    _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[1],
    _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[2],
    _params[UDP_TELEMETRY_PARAM_SERVER_E].data.uint8[3]
  );

  // decide whether to use broadcast or unicast transmission
  // see if we have valid transportAddress?
  if (transportAddress.ipAddress[0] != 0) {
    // unicast 
    sendAddressedPacket(buffer, transportAddress.ipAddress);
  } else {
    // this is probably a hello packet, or similar

    // broadcast
    sendAddressedPacket(buffer, broadcastIp);

    // also send a unicast to our server if we have one
    if (serverIp[0] != 0) {
      sendAddressedPacket(buffer, serverIp);
    }
  }

 return true;
}


void UDPTelemetryModule::sendAddressedPacket(uint8_t *buffer, IPAddress ipAddress) {
  uint8_t txSize = getDroneMeshMsgTotalSize(buffer);

  _udp.beginPacket(ipAddress, _params[UDP_PARAM_PORT_E].data.uint32[0]);
  _udp.write(buffer, txSize);
  _udp.endPacket();

  _packetsSent++;
}
