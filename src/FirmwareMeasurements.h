#ifndef FIRMWARE_MEASUREMENTS_H
#define FIRMWARE_MEASUREMENTS_H
#include <stdint.h>
#include <stddef.h>
namespace telemetry {
enum class Phase:uint8_t { Request, Connect, Encode, Write, Response, Report };
struct Resources { uint32_t freeHeap,minHeap,largestBlock,stackBytes; };
struct MeasurementEvent {
  uint32_t sequence,atMs,elapsedUs;
  Phase phase;bool end;
  Resources resources;
};
struct MeasurementSnapshot {
  uint32_t iterations=0,maxGapMs=0,overruns=0,sequence=0,overwritten=0;
  uint32_t encodeCalls=0,encodeUs=0,encodeMaxUs=0,writeCalls=0,writeUs=0,writeMaxUs=0;
  size_t count=0;
  MeasurementEvent events[64]={};
};
// No pointers to configuration, packets or credentials are retained. Composition
// serializes access; callers receive a bounded, chronological copy.
class Measurements {
  MeasurementSnapshot state;
  uint32_t previous=0;bool started=false;size_t next=0;
public:
  void loop(uint32_t now) {
    if(started) {uint32_t gap=now-previous;if(gap>state.maxGapMs)state.maxGapMs=gap;if(gap>100)++state.overruns;}
    previous=now;started=true;++state.iterations;
  }
  void record(Phase phase,bool end,uint32_t atMs,uint32_t elapsedUs,Resources resources) {
    if(phase==Phase::Encode||phase==Phase::Write) {
      if(!end)return;
      uint32_t &calls=phase==Phase::Encode?state.encodeCalls:state.writeCalls;
      uint32_t &total=phase==Phase::Encode?state.encodeUs:state.writeUs;
      uint32_t &maximum=phase==Phase::Encode?state.encodeMaxUs:state.writeMaxUs;
      ++calls;total+=elapsedUs;if(elapsedUs>maximum)maximum=elapsedUs;return;
    }
    state.events[next]={++state.sequence,atMs,elapsedUs,phase,end,resources};next=(next+1)%64;
    if(state.count<64)++state.count;else ++state.overwritten;
  }
  void snapshot(MeasurementSnapshot &result) const {
    result=state;
    for(size_t i=0;i<state.count;++i)result.events[i]=state.events[(next+64-state.count+i)%64];
  }
  MeasurementSnapshot snapshot() const {MeasurementSnapshot result;snapshot(result);return result;}
};
// Local-only production composition. Clock/network host tests may substitute
// these hooks while exercising the same bounded state above.
void measureMainLoop(uint32_t now);
void measurePhase(Phase phase,bool end,uint32_t elapsedUs);
}
#endif
