/*

Manages wind speed and direction sensors

*/
#ifndef WINDSPEEDDIR_MODULE_H
#define WINDSPEEDDIR_MODULE_H

#include "../DroneModule.h"


class WindSpeedDirModule:  public DroneModule {
protected:
  uint8_t _channel;
  uint8_t _param;
  uint8_t _pin;
public:

  WindSpeedDirModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm);

  virtual void handleLinkMessage(DroneLinkMsg *msg);

  virtual void setup();
  virtual void loop();

  //void setConfig(uint8_t pin, uint8_t channel, uint8_t param);
  //void setPos(uint8_t p);

};

#endif
