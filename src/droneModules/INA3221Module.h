/*

@type          INA3221
@inherits      I2CBase
@category      Input
@description   Manages a 3-channel INA3221 I2C power monitoring module

@config >>>
[INA3221= 4]
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
#ifndef INA3221_MODULE_H
#define INA3221_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"

#include "Beastdevices_INA3221.h"

/*
@I2CAddress        0x40
*/ 
#define INA3221_I2C_ADDRESS  0x40

// @pub 12;f;3;r;current;Current (Amps) for each channel
#define INA3221_PARAM_CURRENT         (I2CBASE_SUBCLASS_PARAM_START+2)

// @pub 13;f;3;r;power;Power (Watts) for each channel
#define INA3221_PARAM_POWER           (I2CBASE_SUBCLASS_PARAM_START+3)

// @pub 14;f;3;r;loadV;Load voltage for each channel
#define INA3221_PARAM_LOADV           (I2CBASE_SUBCLASS_PARAM_START+4)

// @pub 15;f;3;r;cellV;Cell voltage (loadV / cells) for each channel
#define INA3221_PARAM_CELLV           (I2CBASE_SUBCLASS_PARAM_START+5)

// @pub 16;u8;3;r;alarm;Set to 1 when alarm triggered, 0 otherwise - one value per channel
#define INA3221_PARAM_ALARM           (I2CBASE_SUBCLASS_PARAM_START+6)

// @pub 17;u8;3;w;cells;Number of cells (e.g. for a liPo pack) per channel
#define INA3221_PARAM_CELLS           (I2CBASE_SUBCLASS_PARAM_START+7)

// @pub 18;f;3;w;threshold;Threshold voltage below which alarm is triggered (loadV < threshold) per channel
#define INA3221_PARAM_THRESHOLD       (I2CBASE_SUBCLASS_PARAM_START+8)

// @pub 19;f;3;w;shunt;Shunt resistor values for each channel in mOhm (default 100 mOhm)
#define INA3221_PARAM_SHUNT           (I2CBASE_SUBCLASS_PARAM_START+9)

// @pub 20;f;3;r;usage;Cumulative usage in Amp hours
#define INA3221_PARAM_USAGE           (I2CBASE_SUBCLASS_PARAM_START+10)

#define INA3221_PARAM_CURRENT_E         (I2CBASE_PARAM_ENTRIES+0)
#define INA3221_PARAM_POWER_E           (I2CBASE_PARAM_ENTRIES+1)
#define INA3221_PARAM_LOADV_E           (I2CBASE_PARAM_ENTRIES+2)
#define INA3221_PARAM_CELLV_E           (I2CBASE_PARAM_ENTRIES+3)
#define INA3221_PARAM_ALARM_E           (I2CBASE_PARAM_ENTRIES+4)
#define INA3221_PARAM_CELLS_E           (I2CBASE_PARAM_ENTRIES+5)
#define INA3221_PARAM_THRESHOLD_E       (I2CBASE_PARAM_ENTRIES+6)
#define INA3221_PARAM_SHUNT_E           (I2CBASE_PARAM_ENTRIES+7)
#define INA3221_PARAM_USAGE_E           (I2CBASE_PARAM_ENTRIES+8)

#define INA3221_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 9)


// strings
static const char INA3221_STR_INA3221[] PROGMEM = "INA3221";


// class
class INA3221Module:  public I2CBaseModule {
protected:
  Beastdevices_INA3221 *_sensor;

  unsigned long _lastLoopTime;
public:

  INA3221Module(uint8_t id, DroneSystem* ds);
  ~INA3221Module();

  void doReset();

  void publishEntry(uint8_t i);

  void setup();
  void loop();

  uint8_t diagnosticDisplays();
  void drawDiagnosticDisplay(SSD1306Wire *display, uint8_t page);

};

#endif
