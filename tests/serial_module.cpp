#include "droneModules/SerialTelemetryModule.h"
#include "DroneSystem.h"
#include "DroneConsole.h"
#include <cassert>
#include <thread>
Stream Serial,Serial1,Serial2; Logger Log;
std::vector<std::vector<uint8_t>> handoffs;
int main(){
  DroneSystem system; SerialTelemetryModule module(3,&system); Stream port;
  const std::vector<uint8_t> frame={0xfe,0x05,0xfa,0xfa,0x02,0x02,0x18,0x47,0xfa,0x02,0x01,0x0d,0x70,0x00,0x1b};
  // Construction/failed claims and null TX/RX are safe without a port.
  module.loop(); assert(!module.sendPacket(nullptr,{}));
  system.claim=false; module.setup();module.loop();assert(!module.sendPacket(const_cast<uint8_t*>(frame.data()+1),{}));system.claim=true;
  module.setPort(&port); port.input=frame; module.loop();
  assert(handoffs.size()==1); // Must dispatch on the CRC byte, not the following byte.
  assert(handoffs[0]==std::vector<uint8_t>(frame.begin()+1,frame.end()));
  port.input.insert(port.input.end(),frame.begin(),frame.end());module.loop();assert(handoffs.size()==2);
  port.input.insert(port.input.end(),{0xfe,48});module.loop();assert(handoffs.size()==2);
  // Partial input is discarded on a different Stream binding.
  port.input.insert(port.input.end(),frame.begin(),frame.begin()+5);module.loop();
  Stream second;module.setPort(&second);second.input.assign(frame.begin()+5,frame.end());module.loop();assert(handoffs.size()==2);
  second.input.insert(second.input.end(),frame.begin(),frame.end());module.loop();assert(handoffs.size()==3);
  uint8_t invalid=63;assert(!module.sendPacket(&invalid,{}));assert(second.output.empty());
  assert(module.sendPacket(const_cast<uint8_t*>(frame.data()+1),{}));assert(second.output==frame);
  // Real Console Print adapter: UART0 logs are queued/framed, mesh owns its record.
  module.setPort(&Serial);const uint8_t text[]={'h','i','\n'};DroneConsole.write(text,3);
  assert(Serial.output.empty());assert(module.sendPacket(const_cast<uint8_t*>(frame.data()+1),{}));DroneConsole.service();
  assert(std::vector<uint8_t>(Serial.output.begin(),Serial.output.begin()+frame.size())==frame);
  assert(std::vector<uint8_t>(Serial.output.begin()+frame.size(),Serial.output.end())==std::vector<uint8_t>({0xfd,3,3,0,0,104,105,10,0xf7}));
  module.setPort(nullptr);DroneConsole.write(text,3);assert(Serial.output.back()=='\n');
  auto size=Serial.output.size();std::vector<std::thread> workers;
  for(int t=0;t<4;++t) workers.emplace_back([&](){for(int i=0;i<100;++i)DroneConsole.write(text,3);});
  for(auto& worker:workers) worker.join();assert(Serial.output.size()==size+1200);


}
