#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Internal, platform-independent serial framing. Mesh CRC remains owned by DLM.
namespace serialTransport {
static const size_t MaxMesh = 56;
static const size_t MaxText = 48;
inline uint8_t crc8(const uint8_t* bytes, size_t size) {
  uint8_t crc=0;
  for(size_t i=0;i<size;++i) {
    crc ^= bytes[i];
    for(uint8_t bit=0;bit<8;++bit) crc=(crc & 0x80) ? (crc<<1)^7 : crc<<1;
  }
  return crc;
}
inline size_t meshSize(const uint8_t* bytes) {
  if(!bytes || (bytes[0]&63)>=48) return 0;
  return (bytes[0]&63)+9;
}
class MeshDecoder {
 public:
  enum Result { Incomplete, Rejected, Complete };
  void reset() { state=0; used=0; }
  Result consume(uint8_t byte) {
    if(state==0) {if(byte==0xfe) {state=1;used=0;} return Incomplete;}
    if(state==1) {
      length=meshSize(&byte);
      if(!length) {state=byte==0xfe ? 1:0;used=0;return Rejected;}
      state=2;
    }
    buffer[used++]=byte;
    if(used!=length) return Incomplete;
    state=0;return Complete;
  }
  const uint8_t* data() const {return buffer;}
  size_t size() const {return used;}
 private:
  uint8_t buffer[MaxMesh]={};
  size_t length=0,used=0;
  uint8_t state=0;
};
// Platform adapters serialize access to this core. A pending short write retains
// ownership of the wire until completion; no second record can interleave.
class Sink {
 public:
  virtual ~Sink() {}
  virtual size_t capacity() = 0;
  virtual size_t write(const uint8_t*,size_t) = 0;
};
class Writer {
 public:
  void bind(Sink* target) {
    sink=target; head=count=assembled=offset=pendingSize=0;
    sequence=0; lineStart=true; lossEpoch=reportedLoss=0;
  }
  void noteLoss() { ++lossEpoch; }
  void log(const uint8_t* bytes,size_t size) {
    if(!sink) return;
    if(busy) {noteLoss();return;}
    for(size_t i=0;i<size;++i) {
      assembly[assembled++]=bytes[i];
      if(assembled==MaxText || bytes[i]=='\n') enqueue(bytes[i]=='\n');
    }
  }
  bool sendMesh(const uint8_t* bytes) {
    const size_t size=meshSize(bytes);
    if(!sink || !size || busy) return false;
    if(pendingSize) {drain(); if(pendingSize) return false;}
    pending[0]=0xfe;memcpy(pending+1,bytes,size);
    pendingSize=size+1;offset=0;pendingDiagnostic=false;
    drain();return pendingSize==0;
  }
  void service() {
    if(!sink || busy) return;
    if(assembled) enqueue(false);
    if(pendingSize) {drain();return;}
    if(!count) return;
    auto& record=queue[head];
    if(sink->capacity()<record.size) return;
    memcpy(pending,record.bytes,record.size);
    pendingLoss=lossEpoch;
    if(pendingLoss!=reportedLoss) pending[2]|=4;
    pending[record.size-1]=crc8(pending+1,record.size-2);
    pendingSize=record.size;offset=0;pendingDiagnostic=true;
    head=(head+1)%4;--count;
    drain();
  }
 private:
  struct Record {uint8_t bytes[MaxText+6];size_t size;};
  Sink* sink=nullptr;
  Record queue[4];
  uint8_t assembly[MaxText],pending[MaxMesh+1];
  size_t head=0,count=0,assembled=0,pendingSize=0,offset=0;
  uint16_t sequence=0;
  uint32_t lossEpoch=0,reportedLoss=0,pendingLoss=0;
  bool lineStart=true,busy=false,pendingDiagnostic=false;
  void enqueue(bool end) {
    uint16_t current=sequence++;
    if(count==4) noteLoss();
    else {
      auto& record=queue[(head+count)%4];
      record.size=assembled+6;
      record.bytes[0]=0xfd;record.bytes[1]=assembled;
      record.bytes[2]=(lineStart?1:0)|(end?2:0);
      record.bytes[3]=current&255;record.bytes[4]=current>>8;
      memcpy(record.bytes+5,assembly,assembled);
      // CRC is finalized on transmission to include pending loss indication.
      ++count;
    }
    assembled=0;lineStart=end;
  }
  void drain() {
    busy=true;
    size_t written=sink->write(pending+offset,pendingSize-offset);
    if(written>pendingSize-offset) written=pendingSize-offset;
    offset+=written;
    if(offset==pendingSize) {
      if(pendingDiagnostic) reportedLoss=pendingLoss;
      pendingSize=offset=0;
    }
    busy=false;
  }
};
}
