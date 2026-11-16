#include "../DroneConsole.h"
#include "SerialTelemetryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../DroneSystem.h"

// @type SerialTelemetry

SerialTelemetryModule::SerialTelemetryModule(uint8_t id, DroneSystem* ds):
  NetworkInterfaceModule ( id, ds )
 {
   setTypeName(FPSTR(SERIAL_TELEMETRY_STR_SERIAL_TELEMETRY));
   _broadcastCapable = false;

   _packetsReceived = 0;
   _packetsRejected = 0;
   _packetsSent = 0;
   _packetsTimer = 0;

   // pubs
   initParams(SERIAL_TELEMETRY_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[SERIAL_TELEMETRY_PARAM_PORT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SERIAL_TELEMETRY_PARAM_PORT);
   setParamName(FPSTR(STRING_PORT), param);
   _params[SERIAL_TELEMETRY_PARAM_PORT_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[SERIAL_TELEMETRY_PARAM_PORT_E].data.uint8[0] = 1;

   param = &_params[SERIAL_TELEMETRY_PARAM_PACKETS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SERIAL_TELEMETRY_PARAM_PACKETS);
   setParamName(FPSTR(STRING_PACKETS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT32_T, 12);
   param->data.uint32[0] = 0;
   param->data.uint32[1] = 0;
   param->data.uint32[2] = 0;

   param = &_params[SERIAL_TELEMETRY_PARAM_SPEED_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SERIAL_TELEMETRY_PARAM_SPEED);
   setParamName(FPSTR(STRING_SPEED), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 12);
   param->data.f[0] = 0;
   param->data.f[1] = 0;
   param->data.f[2] = 0;

   param = &_params[SERIAL_TELEMETRY_PARAM_BAUD_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, SERIAL_TELEMETRY_PARAM_BAUD);
   setParamName(FPSTR(STRING_BAUD), param);
   _params[SERIAL_TELEMETRY_PARAM_BAUD_E].paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT32_T, 4);
   // @default baud=115200
   _params[SERIAL_TELEMETRY_PARAM_BAUD_E].data.uint32[0] = 115200;
}


uint8_t SerialTelemetryModule::getInterfaceType() {
  // to be overridden
  return DRONE_MESH_INTERFACE_TYPE_SERIAL;
}


void SerialTelemetryModule::setup() {
  DroneModule::setup();

  // request the serial port
  const int logLevel = Log.getLevel();
  if (!_ds->requestSerialPort(_params[SERIAL_TELEMETRY_PARAM_PORT_E].data.uint8[0], this)) {
    setPort(nullptr);
    Log.errorln(F("[Serial.s] Unable to access serial port: %u"), _params[SERIAL_TELEMETRY_PARAM_PORT_E].data.uint8[0]);
    setError(1);
    return;
  }

  switch(_params[SERIAL_TELEMETRY_PARAM_PORT_E].data.uint8[0]) {
    case 0:
      Log.noticeln(F("[Serial.s] Serial 0 at %u"), _params[SERIAL_TELEMETRY_PARAM_BAUD_E].data.uint32[0]);

      // reset serial for telemetry
      Serial.updateBaudRate(_params[SERIAL_TELEMETRY_PARAM_BAUD_E].data.uint32[0]);
      setPort(&Serial);
      Log.setLevel(logLevel);
      break;
    case 1:
      Log.noticeln(F("[Serial.s] Serial 1 at %u"), _params[SERIAL_TELEMETRY_PARAM_BAUD_E].data.uint32[0]);
      Serial1.begin(_params[SERIAL_TELEMETRY_PARAM_BAUD_E].data.uint32[0], SERIAL_8N1, PIN_SERIAL1_RX, PIN_SERIAL1_TX);
      setPort(&Serial1);
      break;
    case 2:
      Log.noticeln(F("[Serial.s] Serial 2 at %u"), _params[SERIAL_TELEMETRY_PARAM_BAUD_E].data.uint32[0]);
      Serial2.begin(_params[SERIAL_TELEMETRY_PARAM_BAUD_E].data.uint32[0], SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX);
      setPort(&Serial2);
      break;
  }

  // register network interface
  _dlm->registerInterface(this);

  _interfaceState = true;
}

void SerialTelemetryModule::loop() {
  DroneModule::loop();

  if (_port) {
    while (_port->available()) {
      int byte = _port->read();
      if (byte < 0) break;
      auto result = _decoder.consume(static_cast<uint8_t>(byte));
      if (result == serialTransport::MeshDecoder::Rejected) ++_packetsRejected;
      if (result == serialTransport::MeshDecoder::Complete) {
        DRONE_LINK_TRANSPORT_ADDRESS address = {};
        receivePacket(const_cast<uint8_t*>(_decoder.data()), 1, address);
        ++_packetsReceived;
      }
    }
  }

  // update and publish packet counters
  if (millis() > _packetsTimer + 5000) {
    uint32_t delta[3];
    delta[0] = _packetsSent - _params[SERIAL_TELEMETRY_PARAM_PACKETS_E].data.uint32[0];
    delta[1] = _packetsReceived - _params[SERIAL_TELEMETRY_PARAM_PACKETS_E].data.uint32[1];
    delta[2] = _packetsRejected - _params[SERIAL_TELEMETRY_PARAM_PACKETS_E].data.uint32[2];


    _params[SERIAL_TELEMETRY_PARAM_PACKETS_E].data.uint32[0] = _packetsSent;
    _params[SERIAL_TELEMETRY_PARAM_PACKETS_E].data.uint32[1] = _packetsReceived;
    _params[SERIAL_TELEMETRY_PARAM_PACKETS_E].data.uint32[2] = _packetsRejected;

    publishParamEntry(&_params[SERIAL_TELEMETRY_PARAM_PACKETS_E]);

    float dur = (millis() - _packetsTimer)/1000.0;
    for (uint8_t i=0; i<3; i++) {
      _params[SERIAL_TELEMETRY_PARAM_SPEED_E].data.f[i] = delta[i] / dur;
    }
    publishParamEntry(&_params[SERIAL_TELEMETRY_PARAM_SPEED_E]);

    _packetsTimer = millis();
  }
}

void SerialTelemetryModule::setPort(Stream *port) {
  if (_port != port) {
    _decoder.reset();
    if (_port == &Serial) DroneConsole.telemetry(false);
    if (port == &Serial) DroneConsole.telemetry(true);
  }
  _port = port;
}


boolean SerialTelemetryModule::sendPacket(uint8_t *buffer, DRONE_LINK_TRANSPORT_ADDRESS) {
  const size_t size = serialTransport::meshSize(buffer);
  if (!_enabled || !_port || !size) return false;
  if (_port == &Serial) {
    if (!DroneConsole.sendMesh(buffer)) return false;
    ++_packetsSent;
    return true;
  }
  uint8_t frame[serialTransport::MaxMesh + 1];
  frame[0] = SERIAL_TELEMETRY_START_OF_FRAME;
  memcpy(frame + 1, buffer, size);
  const size_t written = _port->write(frame, size + 1);
  _port->flush();
  if (written != size + 1) return false;
  ++_packetsSent;
  return true;
}
