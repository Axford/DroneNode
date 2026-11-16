#include "SerialTransport.h"
#include "SerialVectors.h"
#include <cassert>
#include <vector>
using namespace serialTransport;
struct MemorySink : Sink {
  std::vector<uint8_t> bytes; size_t room=128, limit=128;
  size_t capacity() override {return room;}
  size_t write(const uint8_t* b,size_t n) override {n=n<limit?n:limit;bytes.insert(bytes.end(),b,b+n);return n;}
};
int main() {
  MeshDecoder decoder;
  const uint8_t frame[] = {0xfe,0x05,0xfa,0xfa,0x02,0x02,0x18,0x47,0xfa,0x02,0x01,0x0d,0x70,0x00,0x1b};
  int completed = 0;
  for (int repeat=0; repeat<2; ++repeat) {
    for (size_t i=0; i<sizeof(frame); ++i) {
      auto result=decoder.consume(frame[i]);
      assert((result==MeshDecoder::Complete) == (i==sizeof(frame)-1));
      if(result==MeshDecoder::Complete) {++completed; assert(decoder.size()==14); assert(decoder.data()[13]==0x1b);}
    }
  }
  assert(completed==2);
  const std::vector<std::vector<uint8_t>> fixtures={std::vector<uint8_t>(mesh0,mesh0+sizeof(mesh0)),std::vector<uint8_t>(mesh1,mesh1+sizeof(mesh1)),std::vector<uint8_t>(mesh2,mesh2+sizeof(mesh2))};
  for(const auto& fixture:fixtures) for(size_t split=0;split<=fixture.size();++split) {
    decoder.reset();int count=0;
    for(size_t i=0;i<split;++i) if(decoder.consume(fixture[i])==MeshDecoder::Complete)++count;
    for(size_t i=split;i<fixture.size();++i) if(decoder.consume(fixture[i])==MeshDecoder::Complete)++count;
    assert(count==1);assert(std::vector<uint8_t>(decoder.data(),decoder.data()+decoder.size())==std::vector<uint8_t>(fixture.begin()+1,fixture.end()));
    assert(crc8(fixture.data()+1,fixture.size()-2)==fixture.back());
  }

  MemorySink sink; Writer writer; writer.bind(&sink);
  const uint8_t text[]={'h','i','\n'};
  writer.log(text,3); assert(sink.bytes.empty()); writer.service();
  const std::vector<uint8_t> expected={0xfd,3,3,0,0,'h','i','\n',0xf7};
  assert(sink.bytes==expected);
  assert(writer.sendMesh(frame+1));
  assert(std::vector<uint8_t>(sink.bytes.begin()+expected.size(),sink.bytes.end())==std::vector<uint8_t>(frame,frame+sizeof(frame)));

  assert(decoder.consume(0xfe)==MeshDecoder::Incomplete);
  assert(decoder.consume(48)==MeshDecoder::Rejected);
  assert(decoder.consume(0xfe)==MeshDecoder::Incomplete);
  assert(decoder.consume(63)==MeshDecoder::Rejected);
  // Embedded markers and every payload length stay inside the production buffer.
  for(size_t length=1;length<=48;++length) {
    decoder.reset();assert(decoder.consume(0xfe)==MeshDecoder::Incomplete);
    for(size_t i=0;i<length+8;++i) {
      auto result=decoder.consume(i==0?length-1:(i%2?0xfe:0xfd));
      assert((result==MeshDecoder::Complete)==(i==length+7));
    }
    assert(decoder.size()==length+8);
  }
  decoder.reset();decoder.consume(0xfe);assert(decoder.consume(0xfe)==MeshDecoder::Rejected);
  for(size_t i=1;i<sizeof(frame);++i) assert((decoder.consume(frame[i])==MeshDecoder::Complete)==(i==sizeof(frame)-1));
  decoder.reset();for(size_t i=0;i<100000;++i) decoder.consume(static_cast<uint8_t>(i*73));
  const uint8_t check[]={'1','2','3','4','5','6','7','8','9'};assert(crc8(check,9)==0xf4);
  // Backpressure never blocks or starts a diagnostic, and mesh goes first.
  sink.bytes.clear();writer.bind(&sink);sink.room=0;writer.log(text,3);writer.service();assert(sink.bytes.empty());
  assert(writer.sendMesh(frame+1));assert(sink.bytes==std::vector<uint8_t>(frame,frame+sizeof(frame)));
  sink.room=128;writer.service();assert(sink.bytes.size()==sizeof(frame)+9);
  // A partial record keeps ownership, even if another mesh send arrives.
  sink.bytes.clear();writer.bind(&sink);sink.limit=2;writer.log(text,3);writer.service();
  assert(!writer.sendMesh(frame+1));
  for(int i=0;i<8;++i)writer.service();assert(sink.bytes==expected);
  sink.limit=128;
  // Full queue discards new chunks, with sticky LOSS and ordered sequence gaps.
  sink.bytes.clear();writer.bind(&sink);std::vector<uint8_t> longText(48*6,'x');writer.log(longText.data(),longText.size());
  for(int i=0;i<4;++i)writer.service();assert(sink.bytes.size()==54*4);assert(sink.bytes[2]&4);
  assert(!(sink.bytes[54+2]&4));writer.log(text,3);writer.service();assert(sink.bytes[54*4+3]==6);
  // Actual writer sequence wraps across independently pinned UTF-8 fixtures.
  sink.bytes.clear();writer.bind(&sink);const uint8_t newline='\n';
  for(size_t i=0;i<65535;++i) {writer.log(&newline,1);writer.service();sink.bytes.clear();}
  uint8_t lead=0xe2;writer.log(&lead,1);writer.service();
  assert(sink.bytes==std::vector<uint8_t>(diagnostic1,diagnostic1+sizeof(diagnostic1)));
  sink.bytes.clear();const uint8_t tail[]={0x82,0xac,10};writer.log(tail,3);writer.service();
  assert(sink.bytes==std::vector<uint8_t>(diagnostic2,diagnostic2+sizeof(diagnostic2)));
  // Null writer and malformed TX do not dereference or write.
  writer.bind(nullptr);assert(!writer.sendMesh(frame+1));writer.service();writer.log(text,3);
  writer.bind(&sink);uint8_t invalid=63;auto before=sink.bytes.size();assert(!writer.sendMesh(&invalid));assert(sink.bytes.size()==before);
  // Caller reentrancy cannot interleave bytes with the active write.
  struct ReentrantSink : MemorySink {
    Writer* writer; bool nested=false;
    size_t write(const uint8_t* b,size_t n) override {
      if(!nested) {nested=true;const uint8_t log[]={'r','\n'};writer->log(log,2);assert(!writer->sendMesh(b+1));}
      return MemorySink::write(b,n);
    }
  } reentrant;
  writer.bind(&reentrant);reentrant.writer=&writer;assert(writer.sendMesh(frame+1));
  assert(reentrant.bytes==std::vector<uint8_t>(frame,frame+sizeof(frame)));
  writer.log(text,3);writer.service();assert(reentrant.bytes[sizeof(frame)+2]&4);

}
