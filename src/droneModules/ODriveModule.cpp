#include "ODriveModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../pinConfig.h"
#include "strings.h"
#include "DroneSystem.h"

ODriveModule::ODriveModule(uint8_t id, DroneSystem* ds):
  DroneModule ( id, ds )
 {
   setTypeName(FPSTR(ODRIVE_STR_ODRIVE));

   // subs
   initSubs(ODRIVE_SUBS);

   DRONE_PARAM_SUB *sub;

   sub = &_subs[ODRIVE_SUB_LEFT_E];
   sub->addrParam = ODRIVE_SUB_LEFT_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ODRIVE_SUB_LEFT);
   setParamName(FPSTR(STRING_LEFT), &sub->param);

   sub = &_subs[ODRIVE_SUB_RIGHT_E];
   sub->addrParam = ODRIVE_SUB_RIGHT_ADDR;
   sub->param.paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ODRIVE_SUB_RIGHT);
   setParamName(FPSTR(STRING_RIGHT), &sub->param);

   // pubs
   initParams(ODRIVE_PARAM_ENTRIES);

   DRONE_PARAM_ENTRY *param;

   param = &_params[ODRIVE_PARAM_LIMITS_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ODRIVE_PARAM_LIMITS);
   setParamName(FPSTR(STRING_LIMITS), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_FLOAT, 8);
   // @default limits=-1,1
   _params[ODRIVE_PARAM_LIMITS_E].data.f[0] = -1;
   _params[ODRIVE_PARAM_LIMITS_E].data.f[1] = 1;

   param = &_params[ODRIVE_PARAM_PORT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ODRIVE_PARAM_PORT);
   setParamName(FPSTR(STRING_PORT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   // @default port=2
   _params[ODRIVE_PARAM_PORT_E].data.uint8[0] = 2;

   param = &_params[ODRIVE_PARAM_INVERT_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ODRIVE_PARAM_INVERT);
   setParamName(FPSTR(STRING_INVERT), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 2);
   _params[ODRIVE_PARAM_INVERT_E].data.uint8[0] = 0;
   _params[ODRIVE_PARAM_INVERT_E].data.uint8[1] = 0;

   param = &_params[ODRIVE_PARAM_SWITCH_E];
   param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ODRIVE_PARAM_SWITCH);
   setParamName(FPSTR(STRING_SWITCH), param);
   param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
   _params[ODRIVE_PARAM_SWITCH_E].data.uint8[0] = 0;

}


void ODriveModule::setPort(Stream *port) {
  _port = port;
}


void ODriveModule::setup() {
  DroneModule::setup();

  // request the serial port
  if (!_ds->requestSerialPort(_params[ODRIVE_PARAM_PORT_E].data.uint8[0], this)) {
    _port = NULL;
    Log.errorln(F("[ODRIVE.s] Unable to access serial port: %u"), _params[ODRIVE_PARAM_PORT_E].data.uint8[0]);
    setError(1);
    return;
  }

  switch(_params[ODRIVE_PARAM_PORT_E].data.uint8[0]) {
    //case 0: Serial.begin(_baud, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX); break;
    case 1:
      //Log.noticeln(F("[ODRIVE.s] Serial 1 at %u"), _params[ODRIVE_PARAM_BAUD_E].data.uint32[0]);
      Serial1.begin(115200, SERIAL_8N1, PIN_SERIAL1_RX, PIN_SERIAL1_TX);
      setPort(&Serial1);
      break;
    case 2:
      //Log.noticeln(F("[ODRIVE.s] Serial 2 at %u"), _params[ODRIVE_PARAM_BAUD_E].data.uint32[0]);
      Serial2.begin(115200, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX);
      setPort(&Serial2);
      break;
    default:
      _port = NULL;
      Log.errorln(F("[ODRIVE.s] invalid port: %u"), _params[ODRIVE_PARAM_PORT_E].data.uint8[0]);
      setError(1);
  }
}


void ODriveModule::disable() {
  DroneModule::disable();
  _subs[ODRIVE_SUB_LEFT_E].param.data.f[0] = 0;
  _subs[ODRIVE_SUB_RIGHT_E].param.data.f[0] = 0;
  update();
}


void ODriveModule::setVel(uint8_t axis, float v, boolean invert) {

  static char floatStr[10];

  // limit range
  if (v > 1) v = 1;
  if (v< -1) v = -1;

  // remap -1 to 1 into _limits[0] to _limits[1]
  v = (v + 1) * (_params[ODRIVE_PARAM_LIMITS_E].data.f[1] - _params[ODRIVE_PARAM_LIMITS_E].data.f[0]) / (2) + _params[ODRIVE_PARAM_LIMITS_E].data.f[0];

  if (invert) {
    v = -v;
  }

  dtostrf(v,5, 1, floatStr);

  _port->write("v ");
  _port->write(axis == 0 ? "0" : "1");
  _port->write(" ");
  _port->write(floatStr);
  _port->write("\n");

}


void ODriveModule::update() {
  if (_error > 0 || !_setupDone) return;

  boolean swap = _params[ODRIVE_PARAM_SWITCH_E].data.uint8[0] == 1;

  setVel(swap ? 1 : 0, _subs[ODRIVE_SUB_LEFT_E].param.data.f[0], _params[ODRIVE_PARAM_INVERT_E].data.uint8[0]==1 );
  setVel(swap ? 0 : 1, _subs[ODRIVE_SUB_RIGHT_E].param.data.f[0], _params[ODRIVE_PARAM_INVERT_E].data.uint8[1]==1  );
}
