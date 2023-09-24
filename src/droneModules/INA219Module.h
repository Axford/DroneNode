/*

@type          INA219
@inherits      I2CBase
@category      Input
@description   Manages an INA219 I2C power monitoring module

@config >>>
[INA219= 4]
  name= "Power"
  bus= 1
  addr= 64
  interval= 1000
  cells= 2
  threshold= 6.4
  status= 1
  publish =current, power, loadV, alarm, shuntV, busV, cellV
<<<

*/
#ifndef INA219_MODULE_H
#define INA219_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>

/*
@I2CAddress        0x40
@default addr = 64
*/ 
#define INA219_I2C_ADDRESS  0x40

// @pub 10;f;1;r;shuntV;Voltage across shunt
#define INA219_PARAM_SHUNTV          (I2CBASE_SUBCLASS_PARAM_START+0) // 10

// @pub 11;f;1;r;busV;Bus voltage
#define INA219_PARAM_BUSV            (I2CBASE_SUBCLASS_PARAM_START+1)

// @pub 12;f;1;r;current;Current (Amps)
#define INA219_PARAM_CURRENT         (I2CBASE_SUBCLASS_PARAM_START+2)

// @pub 13;f;1;r;power;Power (Watts)
#define INA219_PARAM_POWER           (I2CBASE_SUBCLASS_PARAM_START+3)

// @pub 14;f;1;r;loadV;Load voltage
#define INA219_PARAM_LOADV           (I2CBASE_SUBCLASS_PARAM_START+4)

// @pub 15;f;1;r;cellV;Cell voltage (loadV / cells)
#define INA219_PARAM_CELLV           (I2CBASE_SUBCLASS_PARAM_START+5)

// @pub 16;u8;1;r;alarm;Set to 1 when alarm triggered, 0 otherwise
#define INA219_PARAM_ALARM           (I2CBASE_SUBCLASS_PARAM_START+6)

// @pub 17;u8;1;w;cells;Number of cells (e.g. for a liPo pack)
#define INA219_PARAM_CELLS           (I2CBASE_SUBCLASS_PARAM_START+7)

// @pub 18;f;1;w;threshold;Threshold voltage below which alarm is triggered (loadV < threshold)
#define INA219_PARAM_THRESHOLD       (I2CBASE_SUBCLASS_PARAM_START+8)

// @pub 19;f;1;r;usage;Cumulative usage in Amp hours
#define INA219_PARAM_USAGE           (I2CBASE_SUBCLASS_PARAM_START+9)

#define INA219_PARAM_SHUNTV_E          (I2CBASE_PARAM_ENTRIES+0)
#define INA219_PARAM_BUSV_E            (I2CBASE_PARAM_ENTRIES+1)
#define INA219_PARAM_CURRENT_E         (I2CBASE_PARAM_ENTRIES+2)
#define INA219_PARAM_POWER_E           (I2CBASE_PARAM_ENTRIES+3)
#define INA219_PARAM_LOADV_E           (I2CBASE_PARAM_ENTRIES+4)
#define INA219_PARAM_CELLV_E           (I2CBASE_PARAM_ENTRIES+5)
#define INA219_PARAM_ALARM_E           (I2CBASE_PARAM_ENTRIES+6)
#define INA219_PARAM_CELLS_E           (I2CBASE_PARAM_ENTRIES+7)
#define INA219_PARAM_THRESHOLD_E       (I2CBASE_PARAM_ENTRIES+8)
#define INA219_PARAM_USAGE_E           (I2CBASE_PARAM_ENTRIES+9)

#define INA219_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 10)


// strings
static const char INA219_STR_INA219[] PROGMEM = "INA219";


// class
class INA219Module:  public I2CBaseModule {
protected:
  Adafruit_INA219 *_sensor;

  unsigned long _lastLoopTime;
public:

  INA219Module(uint8_t id, DroneSystem* ds);
  ~INA219Module();

  void doReset();

  void publishEntry(uint8_t i);

  void setup();
  void loop();


};

#endif
