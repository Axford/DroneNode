/*

@type          Inject
@inherits      Drone
@category      Input
@description   Allows a sub value to be injected into a static writable param.  Useful when wanting to dynamically control a value that doesn't support subscribing to an input.

@guide >>>

<<<

@config >>>
[Inject = 5]
  
<<<

*/
#ifndef INJECT_MODULE_H
#define INJECT_MODULE_H

#include "../DroneModule.h"


// pubs

// @pub 10;u8;3;w;target;Target address of the param to modify
#define INJECT_PARAM_TARGET        10
#define INJECT_PARAM_TARGET_E      0


#define INJECT_PARAM_ENTRIES       1

// subs

// @sub 11;12;f;1;input;Input value to sub to
#define INJECT_SUB_INPUT          11
#define INJECT_SUB_INPUT_ADDR     12
#define INJECT_SUB_INPUT_E        0

#define INJECT_SUBS               1


static const char INJECT_STR_INJECT[] PROGMEM = "Inject";

class InjectModule:  public DroneModule {
protected:
    DroneLinkMsg _targetMsg;
public:

  InjectModule(uint8_t id, DroneSystem* ds);
  
  virtual void setup();

  void update();

};

#endif
