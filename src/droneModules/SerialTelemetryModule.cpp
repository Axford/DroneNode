#include "SerialTelemetryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"
#include "../DroneSystem.h"

SerialTelemetryModule::SerialTelemetryModule(uint8_t id, DroneSystem* ds):
  NetworkInterfaceModule ( id, ds )
 {
   setTypeName(FPSTR(SERIAL_TELEMETRY_STR_SERIAL_TELEMETRY));
   _buffer[0] = SERIAL_TELEMETRY_START_OF_FRAME;
   _decodeState = 0;
   _receivedSize = 0;
   _portNum = 2;
   _baud = 57600;

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
   _params[SERIAL_TELEMETRY_PARAM_BAUD_E].data.uint32[0] = 115200;
}


DEM_NAMESPACE* SerialTelemetryModule::registerNamespace(DroneExecutionManager *dem) {
  // namespace for module type
  return dem->createNamespace(SERIAL_TELEMETRY_STR_SERIAL_TELEMETRY,0,true);
}


void SerialTelemetryModule::registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // writable mgmt params
  DEMCommandHandler ph = std::bind(&DroneExecutionManager::mod_param, dem, _1, _2, _3, _4);

  dem->registerCommand(ns, STRING_PORT, DRONE_LINK_MSG_TYPE_UINT8_T, ph);
  dem->registerCommand(ns, STRING_BAUD, DRONE_LINK_MSG_TYPE_UINT32_T, ph);
}


uint8_t SerialTelemetryModule::getInterfaceType() {
  // to be overridden
  return DRONE_MESH_INTERFACE_TYPE_SERIAL;
}


void SerialTelemetryModule::setup() {
  DroneModule::setup();

  // request the serial port
  if (!_ds->requestSerialPort(_params[SERIAL_TELEMETRY_PARAM_PORT_E].data.uint8[0], this)) {
    _port = NULL;
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

  if (_port->available()) {
    //Serial.print("T: ");
    while (_port->available())
    {
      uint8_t b = _port->read();
      if (_receivedSize < sizeof(_buffer))
        _buffer[_receivedSize] = b;

      //Serial.print(b, HEX);

      switch(_decodeState) {
        case 0: // waiting for start
          if (b == SERIAL_TELEMETRY_START_OF_FRAME) {
            _decodeState = 1;
            _receivedSize = 1;
          }
          break;

        case 1: // found start, waiting to confirm payload length
          _msgLen = getDroneMeshMsgTotalSize(&_buffer[1]);
          if (_msgLen < 8 || _msgLen > DRONE_MESH_MSG_MAX_PACKET_SIZE) {
            Log.errorln("[Serial.rP] Invalid size");
            _packetsRejected++;
            _decodeState = 0;
          } else {
            _decodeState = 2;
            _receivedSize++;
          }
          break;

        case 2: // reading payload
          if (_msgLen == _receivedSize - 1) {
            receivePacket(&_buffer[1], 1);
            _packetsReceived++;

            _decodeState = 0;
            _receivedSize = 0;
          }
          _receivedSize++;
          break;
      }
    }
    //Serial.print('\n');
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
  _port = port;
}


boolean SerialTelemetryModule::sendPacket(uint8_t *buffer) {

  if (!_enabled) return false;

  // prefix the DroneMesh message in a start byte
  uint8_t txSize = getDroneMeshMsgTotalSize(buffer) + 1;

  _port->write(SERIAL_TELEMETRY_START_OF_FRAME);
  _port->write(buffer, txSize-1);
  _port->flush();

  _packetsSent++;

  return true;
}
