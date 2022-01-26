/*

@type           Receiver
@inherits       Drone
@description    Reads PWM values from an RC receiver

@config >>>
Receiver.new 14
  name "Receiver"
  interval 200
  pins IN0_0 IN0_1
  limits 980 1020
  //.publish "pins"
  .publish "value1"
  .publish "value2"
  .publish "output"
.done
<<<


*/
#ifndef RECEIVER_MODULE_H
#define RECEIVER_MODULE_H

#include "../DroneModule.h"

// params
// @pub 10;u8;2;pins;Receiver input pins, one per receiver channel (see <a href="pins.html">Pin Mappings</a>)
#define RECEIVER_PARAM_PINS         10
#define RECEIVER_PARAM_PINS_E       0

// @pub 11;f;1;value1;The received value on channel 1 converted to range -1 .. 1
#define RECEIVER_PARAM_VALUE1       11
#define RECEIVER_PARAM_VALUE1_E     1

// @pub 12;f;1;value2;The received value on channel 2 converted to range -1 .. 1
#define RECEIVER_PARAM_VALUE2       12
#define RECEIVER_PARAM_VALUE2_E     2

// @pub 15;u32;2;limits;The PWM timing min and max limits (default 1000 .. 2000)
#define RECEIVER_PARAM_LIMITS       15
#define RECEIVER_PARAM_LIMITS_E     3

// @pub 20;u32;2;output;Raw PWM timing values for each channel
#define RECEIVER_PARAM_OUTPUT       20
#define RECEIVER_PARAM_OUTPUT_E     4

#define RECEIVER_PARAM_ENTRIES      5


// subs

#define RECEIVER_SUBS               0


static const char RECEIVER_STR_RECEIVER[] PROGMEM = "Receiver";

class ReceiverModule:  public DroneModule {
protected:

public:

  ReceiverModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  static void ISR1();
  static void ISR2();

  void setup();
  void update();

  float rawToValue(uint8_t chan);

  void loop();
};

#endif
