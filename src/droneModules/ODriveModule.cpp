#include "ODriveModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "../pinConfig.h"
#include "strings.h"
#include "DroneSystem.h"

// @type ODrive

ODriveModule::ODriveModule(uint8_t id, DroneSystem *ds) : DroneModule(id, ds)
{
  setTypeName(FPSTR(ODRIVE_STR_ODRIVE));

  _bufLen = 0;
  _activeQuery = ODRIVE_QUERY_CURRENT_0;
  _lastSerialFloat = 0;
  _lastSerialHeard = 0;
  _firstSend = true;

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

  param = &_params[ODRIVE_PARAM_MODE_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ODRIVE_PARAM_MODE);
  setParamName(FPSTR(STRING_MODE), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
  _params[ODRIVE_PARAM_MODE_E].data.uint8[0] = ODRIVE_MODE_VELOCITY_CONTROL;

  param = &_params[ODRIVE_PARAM_TORQUE_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, ODRIVE_PARAM_TORQUE);
  setParamName(FPSTR(STRING_TORQUE), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 8);
  _params[ODRIVE_PARAM_TORQUE_E].data.f[0] = 0;
  _params[ODRIVE_PARAM_TORQUE_E].data.f[1] = 0;

  param = &_params[ODRIVE_PARAM_SAMPLES_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_LOW, ODRIVE_PARAM_SAMPLES);
  setParamName(FPSTR(STRING_SAMPLES), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(true, DRONE_LINK_MSG_TYPE_UINT8_T, 1);
  _params[ODRIVE_PARAM_SAMPLES_E].data.uint8[0] = 5;

  param = &_params[ODRIVE_PARAM_ERRORS_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, ODRIVE_PARAM_ERRORS);
  setParamName(FPSTR(STRING_ERRORS), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_FLOAT, 8);
  _params[ODRIVE_PARAM_ERRORS_E].data.f[0] = 0;
  _params[ODRIVE_PARAM_ERRORS_E].data.f[1] = 0;

  param = &_params[ODRIVE_PARAM_STATE_E];
  param->paramPriority = setDroneLinkMsgPriorityParam(DRONE_LINK_MSG_PRIORITY_CRITICAL, ODRIVE_PARAM_STATE);
  setParamName(FPSTR(STRING_STATE), param);
  param->paramTypeLength = _mgmtMsg.packParamLength(false, DRONE_LINK_MSG_TYPE_UINT8_T, 2);
  _params[ODRIVE_PARAM_STATE_E].data.uint8[0] = 0;
  _params[ODRIVE_PARAM_STATE_E].data.uint8[1] = 0;
}

void ODriveModule::setPort(Stream *port)
{
  _port = port;
}

void ODriveModule::setup()
{
  DroneModule::setup();

  // request the serial port
  if (!_ds->requestSerialPort(_params[ODRIVE_PARAM_PORT_E].data.uint8[0], this))
  {
    _port = NULL;
    Log.errorln(F("[ODRIVE.s] Unable to access serial port: %u"), _params[ODRIVE_PARAM_PORT_E].data.uint8[0]);
    setError(1);
    return;
  }

  switch (_params[ODRIVE_PARAM_PORT_E].data.uint8[0])
  {
  // case 0: Serial.begin(_baud, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX); break;
  case 1:
    // Log.noticeln(F("[ODRIVE.s] Serial 1 at %u"), _params[ODRIVE_PARAM_BAUD_E].data.uint32[0]);
    Serial1.begin(115200, SERIAL_8N1, PIN_SERIAL1_RX, PIN_SERIAL1_TX);
    setPort(&Serial1);
    break;
  case 2:
    // Log.noticeln(F("[ODRIVE.s] Serial 2 at %u"), _params[ODRIVE_PARAM_BAUD_E].data.uint32[0]);
    Serial2.begin(115200, SERIAL_8N1, PIN_SERIAL2_RX, PIN_SERIAL2_TX);
    setPort(&Serial2);
    break;
  default:
    _port = NULL;
    Log.errorln(F("[ODRIVE.s] invalid port: %u"), _params[ODRIVE_PARAM_PORT_E].data.uint8[0]);
    setError(1);
  }


  if (_error == 0) {
    // set default position 
    //_port->write("p 0 0\n");
    //_port->write("p 1 0\n");

    // set current absolute position to zero
    //es motor abs_pos
    //_port->write("esl 0 0\n");
    //_port->write("esl 1 0\n");
  }
}

void ODriveModule::disable()
{
  DroneModule::disable();

  if (_params[ODRIVE_PARAM_MODE_E].data.uint8[0] == ODRIVE_MODE_VELOCITY_CONTROL)
  {
    _subs[ODRIVE_SUB_LEFT_E].param.data.f[0] = 0;
    _subs[ODRIVE_SUB_RIGHT_E].param.data.f[0] = 0;

    update();
  }
}

void ODriveModule::setVel(uint8_t axis, float v, boolean invert)
{

  static char floatStr[10];

  uint8_t mode = _params[ODRIVE_PARAM_MODE_E].data.uint8[0];

  if (mode == ODRIVE_MODE_VELOCITY_CONTROL)
  {
    // limit range
    if (v > 1)
      v = 1;
    if (v < -1)
      v = -1;

    // remap -1 to 1 into _limits[0] to _limits[1]
    v = (v + 1) * (_params[ODRIVE_PARAM_LIMITS_E].data.f[1] - _params[ODRIVE_PARAM_LIMITS_E].data.f[0]) / (2) + _params[ODRIVE_PARAM_LIMITS_E].data.f[0];

    if (invert)
    {
      v = -v;
    }

    _port->write("v ");
    _port->write(axis == 0 ? "0" : "1");
    _port->write(" ");
    dtostrf(v, 5, 1, floatStr);
    _port->write(floatStr);
    _port->write("\n");

    // for debug, write the same to the normal serial port
    /*
    Serial.write("v ");
    Serial.write(axis == 0 ? "0" : "1");
    Serial.write(" ");
    dtostrf(v, 5, 1, floatStr);
    Serial.write(floatStr);
    Serial.write("\n");
    */
  }
  else
  {
    // position control

    if (invert)
    {
      v = -v;
    }

    // position target
    dtostrf(v, 6, 3, floatStr);

    if (_firstSend) {
      // set current absolute position to match new target, so motors don't spin on startup
      _port->write("es ");  
      _port->write(axis == 0 ? "0" : "1");  
      _port->write(" ");
      _port->write(floatStr);
      _port->write("\n");
    }

    _port->write("q ");
    _port->write(axis == 0 ? "0" : "1");
    _port->write(" ");
    _port->write(floatStr);
    _port->write(" ");
    dtostrf(_params[ODRIVE_PARAM_LIMITS_E].data.f[0], 6, 2, floatStr);
    _port->write(floatStr);

    _port->write(" ");
    dtostrf(_params[ODRIVE_PARAM_LIMITS_E].data.f[1], 6, 2, floatStr);
    _port->write(floatStr);

    _port->write("\n");

    // for debug, write the same to the normal serial port
    /*
    Serial.write("p ");
    Serial.write(axis == 0 ? "0" : "1");
    Serial.write(" ");
    // position target
    dtostrf(v, 5, 1, floatStr);
    Serial.write(floatStr);

    Serial.write(" ");
    dtostrf(_params[ODRIVE_PARAM_LIMITS_E].data.f[0], 5, 1, floatStr);
    Serial.write(floatStr);

    Serial.write(" ");
    dtostrf(_params[ODRIVE_PARAM_LIMITS_E].data.f[1], 5, 1, floatStr);
    Serial.write(floatStr);

    Serial.write("\n");
    */
  }
}

void ODriveModule::update()
{
  if (_error > 0 || !_setupDone)
    return;

  boolean swap = _params[ODRIVE_PARAM_SWITCH_E].data.uint8[0] == 1;

  // only send commands if state is 8 = closed loop control
  if (_states[0] == 8 && _states[1] == 8) {
    setVel(swap ? 1 : 0, _subs[ODRIVE_SUB_LEFT_E].param.data.f[0], _params[ODRIVE_PARAM_INVERT_E].data.uint8[0] == 1);
    setVel(swap ? 0 : 1, _subs[ODRIVE_SUB_RIGHT_E].param.data.f[0], _params[ODRIVE_PARAM_INVERT_E].data.uint8[1] == 1);

    _firstSend = false;
  }
}

void ODriveModule::generateNextSerialQuery()
{
  switch (_activeQuery)
  {
  case ODRIVE_QUERY_CURRENT_0:
    _port->write("r axis0.motor.current_control.Iq_measured\n");
    //Serial.println("r axis0.Iq");
    break;
  case ODRIVE_QUERY_CURRENT_1:
    _port->write("r axis1.motor.current_control.Iq_measured\n");
    //Serial.println("r axis1.Iq");
    break;
  case ODRIVE_QUERY_ERROR_0:
    _port->write("r axis0.error\n");
    //Serial.println("r axis0.error");
    break;
  case ODRIVE_QUERY_ERROR_1:
    _port->write("r axis1.error\n");
    //Serial.println("r axis1.error");
    break;
  case ODRIVE_QUERY_STATE_0:
    _port->write("r axis0.current_state\n");
    break;
  case ODRIVE_QUERY_STATE_1:
    _port->write("r axis1.current_state\n");
    break;
  }
}

void ODriveModule::manageODriveSerial()
{
  boolean _thatsEnough = false;

  while (_port->available() && !_thatsEnough)
  {
    _lastSerialHeard = _lastLoop;
    char c = _port->read();
    Serial.write(c);
    if ((c == '\n' || c == '\r'))
    {
      if (_bufLen > 0)
      {
        // null terminate
        _buf[_bufLen] = 0;

        // attempt to convert to float
        //Serial.print("Converting to float: [");
        //Serial.print(_buf);
        //Serial.print(']');

        _lastSerialFloat = atof(_buf);
        //Serial.print("=");
        //Serial.println(_lastSerialFloat);

        // do something with the float we received
        switch (_activeQuery)
        {
        case ODRIVE_QUERY_CURRENT_0:
          if (_lastSerialFloat > -100 && _lastSerialFloat < 100) {
            _motorCurrents[0] = (_lastSerialFloat + _params[ODRIVE_PARAM_SAMPLES_E].data.uint8[0] * _params[ODRIVE_PARAM_TORQUE_E].data.f[0]) / (_params[ODRIVE_PARAM_SAMPLES_E].data.uint8[0] + 1);
          }
          break;
        case ODRIVE_QUERY_CURRENT_1:
          if (_lastSerialFloat > -100 && _lastSerialFloat < 100) {
            _motorCurrents[1] = (_lastSerialFloat + _params[ODRIVE_PARAM_SAMPLES_E].data.uint8[0] * _params[ODRIVE_PARAM_TORQUE_E].data.f[1]) / (_params[ODRIVE_PARAM_SAMPLES_E].data.uint8[0] + 1);
            updateAndPublishParam(&_params[ODRIVE_PARAM_TORQUE_E], (uint8_t *)&_motorCurrents, sizeof(_motorCurrents));
          }
          break;
        case ODRIVE_QUERY_ERROR_0:
          _errors[0] = _lastSerialFloat;
          break;
        case ODRIVE_QUERY_ERROR_1:
          _errors[1] = _lastSerialFloat;
          updateAndPublishParam(&_params[ODRIVE_PARAM_ERRORS_E], (uint8_t *)&_errors, sizeof(_errors));
          break;

        case ODRIVE_QUERY_STATE_0:
          _states[0] = _lastSerialFloat;
          break;
        case ODRIVE_QUERY_STATE_1:
          _states[1] = _lastSerialFloat;
          updateAndPublishParam(&_params[ODRIVE_PARAM_STATE_E], (uint8_t *)&_states, sizeof(_states));
          break;
        }

        // select and generate next query
        _activeQuery++;
        if (_activeQuery > 5)
          _activeQuery = 0;

        // only send another query if we've definitely received everything, otherwise we might get out of sync with replies!
        if (!_port->available()) generateNextSerialQuery();
        _thatsEnough = true;
      }

      // reset buffer
      _bufLen = 0;
    }
    else
    {
      _buf[_bufLen] = c;
      if (_bufLen < 29)
        _bufLen++;
    }
  }

  // check if we haven't heard anything for a while and give the serial a tickle
  // also includes the first query on startup
  if (_lastLoop - _lastSerialHeard > 50)
  {
    generateNextSerialQuery();
    _lastSerialHeard = _lastLoop; // make sure we don't spam the ODrive
  }
}

void ODriveModule::loop()
{
  DroneModule::loop();

  // tickle motor watchdog
  //_port->write("u 0\n");
  //_port->write("u 1\n");

  manageODriveSerial();
}