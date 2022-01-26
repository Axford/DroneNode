/*

@type           Receiver
@inherits       Drone
@description    Reads up to 4 PWM channels from an RC receiver

@config >>>
Receiver.new 14
  name "Receiver"
  interval 200
  pins IN0_0 IN0_1 32 0
  limits 980 2020
  $switch [@>14.13]
  $input1 [@>9.14]
  $input2 [@>8.16]
  //.publish "pins"
  .publish "value1"
  .publish "value2"
  .publish "value3"
  .publish "input"
  .publish "limits"
  .publish "input1"
  .publish "input2"
  .publish "switch"
.done
<<<

@guide >>>
<p>PWM values from an RC receiver are decoded using interrupt routines, then converted
and published as values (<b>value1, value2</b>, etc) in range -1..1 on a regular <b>interval</b>.  Output values can be subscribed to from other modules (e.g. Servo or TankSteer).  Up to 4 channels can be read.</p>

<p>The valid range of PWM input values (i.e. pulse timings in ms) are defined by the <b>limits</b> parameter.</p>

<p>Deadman protection is triggered after 5 seconds of no valid PWM signals and all output values are set to zero.</p>

<p>The module can optionally be chained behind other modules (e.g. for automated navigation) with a virtual switch to flip outputs between channel subscriptions (passthrough mode) vs RC receiver inputs.  The <b>switch</b> parameter controls which mode the module is in - if switch is less than 0.5 then it is in passthrough (subscription values are output), otherwise in active mode (i.e. RC receiver values are output). </p>
<<<

*/
#ifndef RECEIVER_MODULE_H
#define RECEIVER_MODULE_H

#include "../DroneModule.h"

// params
// @pub 10;u8;2;pins;Receiver input pins, one per receiver channel (see <a href="pins.html">Pin Mappings</a>)
#define RECEIVER_PARAM_PINS         10
#define RECEIVER_PARAM_PINS_E       0

// @pub 11;f;1;value1;The output value on channel 1 (in range -1 .. 1)
#define RECEIVER_PARAM_VALUE1       11
#define RECEIVER_PARAM_VALUE1_E     1

// @pub 12;f;1;value2;The output value on channel 2 (in range -1 .. 1)
#define RECEIVER_PARAM_VALUE2       12
#define RECEIVER_PARAM_VALUE2_E     2

// @pub 13;f;1;value3;The output value on channel 3 (in range -1 .. 1)
#define RECEIVER_PARAM_VALUE3       13
#define RECEIVER_PARAM_VALUE3_E     3

// @pub 14;f;1;value3;The output value on channel 4 (in range -1 .. 1)
#define RECEIVER_PARAM_VALUE4       14
#define RECEIVER_PARAM_VALUE4_E     4

// @pub 15;u32;2;limits;The PWM timing min and max limits (default 1000 .. 2000)
#define RECEIVER_PARAM_LIMITS       15
#define RECEIVER_PARAM_LIMITS_E     5

// @pub 20;u32;2;input;Raw PWM timing input values for each channel
#define RECEIVER_PARAM_INPUT        20
#define RECEIVER_PARAM_INPUT_E      6

#define RECEIVER_PARAM_ENTRIES      7


// subs

// @sub 21;25;f;1;input1;Subscription for channel 1 (used in passthrough mode)
#define RECEIVER_SUB_INPUT1            21
#define RECEIVER_SUB_INPUT1_ADDR       25
#define RECEIVER_SUB_INPUT1_E          0

// @sub 22;26;f;1;input2;Subscription for channel 2 (used in passthrough mode)
#define RECEIVER_SUB_INPUT2            22
#define RECEIVER_SUB_INPUT2_ADDR       26
#define RECEIVER_SUB_INPUT2_E          1

// @sub 23;27;f;1;input3;Subscription for channel 3 (used in passthrough mode)
#define RECEIVER_SUB_INPUT3            23
#define RECEIVER_SUB_INPUT3_ADDR       27
#define RECEIVER_SUB_INPUT3_E          2

// @sub 24;28;f;1;input4;Subscription for channel 4 (used in passthrough mode)
#define RECEIVER_SUB_INPUT4            24
#define RECEIVER_SUB_INPUT4_ADDR       28
#define RECEIVER_SUB_INPUT4_E          3

// @sub 29;30;f;1;switch;Mode control value - passthrough if less than 0.5, otherwise active (defaults to active)
#define RECEIVER_SUB_SWITCH            29
#define RECEIVER_SUB_SWITCH_ADDR       30
#define RECEIVER_SUB_SWITCH_E          4

#define RECEIVER_SUBS                  5


static const char RECEIVER_STR_RECEIVER[] PROGMEM = "Receiver";

class ReceiverModule:  public DroneModule {
protected:
  unsigned long _lastSignal;

public:

  ReceiverModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs);

  static DEM_NAMESPACE* registerNamespace(DroneExecutionManager *dem);
  static void registerParams(DEM_NAMESPACE* ns, DroneExecutionManager *dem);

  static void ISR1();
  static void ISR2();
  static void ISR3();
  static void ISR4();

  void setup();
  void update();

  float rawToValue(uint8_t chan);

  void loop();
};

#endif
