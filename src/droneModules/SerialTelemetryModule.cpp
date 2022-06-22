#include "SerialTelemetryModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

SerialTelemetryModule::SerialTelemetryModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  NetworkInterfaceModule ( id, dmm, dlm, dem, fs )
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

  switch(_params[SERIAL_TELEMETRY_PARAM_PORT_E].data.uint8[0]) {
    case 0:
      Log.noticeln(F("[Serial.s] Serial 0 at %u"), _params[SERIAL_TELEMETRY_PARAM_BAUD_E].data.uint32[0]);
      // disable logging to serial
      Log.begin(LOG_LEVEL_SILENT, &Serial);
      // reset serial for telemetry
      Serial.begin(_params[SERIAL_TELEMETRY_PARAM_BAUD_E].data.uint32[0], SERIAL_8N1, PIN_SERIAL0_RX, PIN_SERIAL0_TX);
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
    default:
      _port = NULL;
      Log.errorln(F("[Serial.s] invalid port: %u"), _params[SERIAL_TELEMETRY_PARAM_PORT_E].data.uint8[0]);
      setError(1);
  }
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
            _decodeState =3;
          }
          _receivedSize++;
          break;

        case 3: // checking CRC
          uint8_t crc = _CRC8.smbus(_buffer, _receivedSize-1);
          if (crc == _buffer[_receivedSize]) {
            receivePacket(&_buffer[1], 1);
            _packetsReceived++;
          } else {
            Log.errorln("[Serial.rP] CRC fail");
            _packetsRejected++;
          }
          _decodeState = 0;
          _receivedSize = 0;
          break;
      }
    }
    //Serial.print('\n');
  }
}

void SerialTelemetryModule::setPort(Stream *port) {
  _port = port;
}


boolean SerialTelemetryModule::sendPacket(uint8_t *buffer) {

  if (!_enabled) return false;

  // wrap the DroneMesh message in a start byte and end CRC
  uint8_t txSize = getDroneMeshMsgTotalSize(buffer) + 2;


  memcpy(_buffer + 1, buffer, txSize-2);
  _buffer[0] = SERIAL_TELEMETRY_START_OF_FRAME; // ensure this is set, given we reuse the buffer
  _buffer[txSize-1] = _CRC8.smbus(_buffer, txSize - 1);

  /*
  Log.noticeln("[RFM.sP] sending %u bytes", txSize);
  for (uint8_t i=0; i<txSize; i++) {
    Serial.print("  ");
    Serial.print(_buffer[i], BIN);
  }
  Serial.println();
  */

  _port->write(_buffer, txSize);

  _packetsSent++;

  return true;
}
