#ifndef TELEMETRY_REPORT_H
#define TELEMETRY_REPORT_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Network-independent bounded latest-observation policy. Call on one owner thread.
namespace telemetry {
constexpr size_t AddressLimit = 64, FacetLimit = 128, BatchLimit = 32;
struct Packet { uint8_t bytes[21] = {}; uint8_t size = 0; };
struct CycleEntry { Packet packet; uint64_t version = 0; uint8_t slot = 0; };
struct Cycle { CycleEntry entries[FacetLimit]; size_t count = 0; uint32_t generation = 0; };
inline bool publication(const uint8_t* p, size_t size) {
  if (!p || size < 6 || size > 21) return false;
  const unsigned length = (p[4]&15)+1, type = (p[4]>>4)&7;
  return size == 5+length && type <= 5 && (!(type>=1 && type<=3) || length%4==0)
      && p[1]>0 && p[1]<255 && p[2]>0 && p[2]<255 && (p[3]&63)>0
      && (p[0]==0 || p[0]==p[1]);
}
class Report {
  struct Facet { Packet packet; uint64_t version = 0; bool pending = false; };
  struct Address { uint8_t node=0, channel=0, param=0; Facet facets[2]; };
  Address addresses[AddressLimit];
  uint64_t version = 0;
  uint32_t generation = 0, interval = 10000, next = 0;
  size_t used = 0, cursor = 0;
  bool enabled = false;
public:
  uint32_t overflow = 0, captured = 0, coalesced = 0;
  static bool validInterval(uint32_t value) { return value>=1000 && value<=86400000; }
  void start(uint32_t now, uint32_t rate) { stop(); if(!validInterval(rate)) return; enabled=true; interval=rate; next=now+rate; }
  void stop() { enabled=false; ++generation; used=cursor=0; for(auto &a:addresses) a=Address(); }
  void setInterval(uint32_t now,uint32_t rate) { if(validInterval(rate)) { interval=rate; next=now+rate; } }
  void skipTicks(uint32_t now) {
    if (int32_t(now-next)>=0) next += ((now-next)/interval+1)*interval;
  }
  size_t tracked() const { return used; }
  bool capture(const uint8_t* p,size_t size) {
    if (!enabled || !publication(p,size)) return false;
    size_t index=0;
    while(index<used && !(addresses[index].node==p[1] && addresses[index].channel==p[2] && addresses[index].param==(p[3]&63))) ++index;
    if(index==used) {
      if(used==AddressLimit) { ++overflow; return false; }
      addresses[index].node=p[1]; addresses[index].channel=p[2]; addresses[index].param=p[3]&63; ++used;
    }
    Facet &f=addresses[index].facets[((p[4]>>4)&7)==5 ? 1:0];
    if(f.pending) ++coalesced;
    memcpy(f.packet.bytes,p,size); f.packet.size=size; f.version=++version; f.pending=true; ++captured;
    return true;
  }
  void discardOlder(const uint8_t* p) {
    for (size_t i=0;i<used;++i) {
      auto& a=addresses[i];
      if (a.node==p[1] && a.channel==p[2] && a.param==(p[3]&63)) {
        a.facets[((p[4]>>4)&7)==5 ? 1:0].pending=false;
        return;
      }
    }
  }
  bool begin(uint32_t now,bool connected,Cycle &cycle) {
    return begin(now,connected,cycle,[](const Packet&) { return true; });
  }
  template<typename Selection>
  bool begin(uint32_t now,bool connected,Cycle &cycle,Selection eligible) {
    if(!enabled || int32_t(now-next)<0) return false;
    skipTicks(now);
    if(!connected) return false;
    cycle.count=0; cycle.generation=generation;
    for(size_t n=0;n<FacetLimit;++n) {
      const size_t slot=(cursor+n)%FacetLimit;
      const Facet &f=addresses[slot/2].facets[slot%2];
      if(f.pending && eligible(f.packet)) { auto &e=cycle.entries[cycle.count++]; e.packet=f.packet; e.version=f.version; e.slot=slot; }
    }
    return cycle.count>0;
  }
  bool finish(const Cycle &cycle,size_t attempted) {
    if(!enabled || cycle.generation!=generation) return false;
    if(attempted>cycle.count) attempted=cycle.count;
    for(size_t i=0;i<attempted;++i) {
      const auto &e=cycle.entries[i]; auto &f=addresses[e.slot/2].facets[e.slot%2];
      if(f.version==e.version) f.pending=false;
      cursor=(e.slot+1)%FacetLimit;
    }
    return true;
  }
};
}
#endif
