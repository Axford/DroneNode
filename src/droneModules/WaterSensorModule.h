/*

@type          Analog
@inherits      Drone
@description   Manages a simple Analog sensor connected to an analog input pin

@guide >>>
Reads an analog value on specified pin (range 0..4095) and maps into the range limits[min..max]
<<<

@config >>>
[Analog = 5]
  pins = 34
  limits = 0, 10
  publish = raw, analog
<<<

*/
#ifndef WATER_SENSOR_MODULE_H
#define WATER_SENSOR_MODULE_H

#include "../DroneModule.h"


// pubs

// @pub 10;u8;2;pins;Pins for the sensor: {output pin}, {analog pin}
#define WATER_SENSOR_PARAM_PINS         10
#define WATER_SENSOR_PARAM_PINS_E       0

// @pub 11;u32;1;raw;Raw analog reading (0..4095)
#define WATER_SENSOR_PARAM_RAW          11
#define WATER_SENSOR_PARAM_RAW_E        1

// @pub 12;u32;1;threshold;Level over which the alarm is triggered
#define WATER_SENSOR_PARAM_THRESHOLD    12
#define WATER_SENSOR_PARAM_THRESHOLD_E   2

// @pub 13;f;1;alarm;Alarm value, 0=idle, 1=triggered
#define WATER_SENSOR_PARAM_ALARM        13
#define WATER_SENSOR_PARAM_ALARM_E      3

#define WATER_SENSOR_PARAM_ENTRIES      4

// subs
#define WATER_SENSOR_SUBS               0


static const char WATER_SENSOR_STR_WATER_SENSOR[] PROGMEM = "WaterSensor";

#define WATER_SENSOR_SAMPLE_INTERVAL       1000  // between samples
#define WATER_SENSOR_SAMPLE_TRIGGER_TIME   10    // how long to activate trigger


class WaterSensorModule:  public DroneModule {
protected:
  boolean _triggered;
  unsigned long _triggerTime;
public:

  WaterSensorModule(uint8_t id, DroneSystem* ds);
  
  virtual void setup();

  void loop();

};

#endif
