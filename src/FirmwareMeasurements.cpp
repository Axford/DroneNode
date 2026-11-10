#include "FirmwareMeasurements.h"
#include <Arduino.h>
#include <new>
#include <memory>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <esp_heap_caps.h>
#include <ESPAsyncWebServer.h>
namespace telemetry {
namespace {
Measurements measurements;
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
const char* phaseName(Phase phase) {
  switch(phase) {
    case Phase::Request:return "request";case Phase::Connect:return "dnsTcpTlsOrReuse";
    case Phase::Encode:return "encodeChunk";case Phase::Write:return "writeChunk";
    case Phase::Response:return "response";case Phase::Report:return "report";
  }
  return "unknown";
}
Resources resources(bool worker) {
  return {ESP.getFreeHeap(),ESP.getMinFreeHeap(),uint32_t(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
    worker?uint32_t(uxTaskGetStackHighWaterMark(nullptr)):0};
}
// Own one numeric snapshot until the async response finishes or disconnects.
// Format a bounded fragment at a time; avoid StreamString's per-byte front
// removal when the web library drains a populated response stream.
struct MeasurementResponse {
  MeasurementSnapshot snapshot;
  Resources current{};
  uint32_t uptime=0;
  char fragment[768];
  size_t length=0,offset=0,event=0;
  unsigned stage=0;
  bool next() {
    int size=0;
    if(stage==0) {
      const auto &s=snapshot;
      size=snprintf(fragment,sizeof fragment,"{\"schemaVersion\":1,\"uptimeMs\":%u,\"iterations\":%u,\"maxGapMs\":%u,\"overruns\":%u,\"sequence\":%u,\"overwritten\":%u,\"freeHeap\":%u,\"minHeap\":%u,\"largestBlock\":%u,\"encodeCalls\":%u,\"encodeUs\":%u,\"encodeMaxUs\":%u,\"writeCalls\":%u,\"writeUs\":%u,\"writeMaxUs\":%u,\"events\":[",
        uptime,s.iterations,s.maxGapMs,s.overruns,s.sequence,s.overwritten,current.freeHeap,current.minHeap,current.largestBlock,s.encodeCalls,s.encodeUs,s.encodeMaxUs,s.writeCalls,s.writeUs,s.writeMaxUs);
      stage=1;
    } else if(stage==1&&event<snapshot.count) {
      const auto &e=snapshot.events[event];
      size=snprintf(fragment,sizeof fragment,"%s{\"sequence\":%u,\"atMs\":%u,\"phase\":\"%s\",\"end\":%s,\"elapsedUs\":%u,\"freeHeap\":%u,\"minHeap\":%u,\"largestBlock\":%u,\"stackBytes\":%u}",event?",":"",e.sequence,e.atMs,phaseName(e.phase),e.end?"true":"false",e.elapsedUs,e.resources.freeHeap,e.resources.minHeap,e.resources.largestBlock,e.resources.stackBytes);
      ++event;
    } else if(stage==1) {
      memcpy(fragment,"]}",2);size=2;stage=2;
    } else return false;
    if(size<0||size_t(size)>=sizeof fragment) {stage=2;return false;}
    offset=0;length=size_t(size);return true;
  }
  size_t read(uint8_t* output,size_t capacity) {
    size_t written=0;
    while(written<capacity) {
      if(offset==length&&!next()) break;
      const size_t count=std::min(capacity-written,length-offset);
      memcpy(output+written,fragment+offset,count);offset+=count;written+=count;
    }
    return written;
  }
};

}
void measureMainLoop(uint32_t now) {
  portENTER_CRITICAL(&mux);measurements.loop(now);portEXIT_CRITICAL(&mux);
}
void measurePhase(Phase phase,bool end,uint32_t elapsedUs) {
  const bool chunk=phase==Phase::Encode||phase==Phase::Write;
  if(chunk&&!end)return;
  const auto sample=chunk?Resources{}:resources(true);const uint32_t now=millis();
  portENTER_CRITICAL(&mux);measurements.record(phase,end,now,elapsedUs,sample);portEXIT_CRITICAL(&mux);
}
void mountMeasurements(AsyncWebServer &server) {
  server.on("/measurements",HTTP_GET,[](AsyncWebServerRequest* request) {
    // Allocate off the small async stack. Snapshot briefly under the lock;
    // heap sampling, formatting and socket writes remain outside it.
    std::shared_ptr<MeasurementResponse> copy(new(std::nothrow) MeasurementResponse);
    if(!copy) {request->send(503,"text/plain","measurement snapshot unavailable");return;}
    portENTER_CRITICAL(&mux);measurements.snapshot(copy->snapshot);portEXIT_CRITICAL(&mux);
    copy->current=resources(false);copy->uptime=millis();
    auto response=request->beginChunkedResponse("application/json",[copy](uint8_t* output,size_t capacity,size_t) {
      return copy->read(output,capacity);
    });
    if(!response) {request->send(503,"text/plain","measurement response unavailable");return;}
    response->addHeader("Cache-Control","no-store");request->send(response);
  });
}
}
