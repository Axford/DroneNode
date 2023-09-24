/*

@type          Diagnostic
@inherits      I2CBaseModule
@category      Output
@description   Manages a Diagnostic 128x64 I2C display (1306 driver)

@config >>>
[Diagnostic = 6]
  name= "Diagnostic"
  interval= 200
  bus= 0
<<<

*/
#ifndef DIAGNOSTIC_MODULE_H
#define DIAGNOSTIC_MODULE_H

#include "../DroneModule.h"
#include "../DroneWire.h"
#include "I2CBaseModule.h"
#include "LinkedList.h"

#include "I2Cdev.h"
#include "SSD1306Wire.h"

/*
@I2CAddress        0x3C
@default addr = 60
*/ 
#define DIAGNOSTIC_I2C_ADDRESS  0x3C

// pubs
//#define DIAGNOSTIC_PARAM_VECTOR          (I2CBASE_SUBCLASS_PARAM_START+0)


#define DIAGNOSTIC_PARAM_ENTRIES           (I2CBASE_PARAM_ENTRIES + 0)

// subs

#define DIAGNOSTIC_SUBS                      0

// strings
static const char DIAGNOSTIC_STR_DIAGNOSTIC[] PROGMEM = "Diagnostic";


// class
class DiagnosticModule:  public I2CBaseModule {
protected:
  SSD1306Wire *_display;
  uint8_t _currentModule;
  uint8_t _currentPage;
  IvanLinkedList::LinkedList<DroneModule*> _mods; // modules to display
public:

  DiagnosticModule(uint8_t id, DroneSystem* ds);
  ~DiagnosticModule();

  void doReset();
  
  void doShutdown();

  void setup();
  void loop();


};

#endif
