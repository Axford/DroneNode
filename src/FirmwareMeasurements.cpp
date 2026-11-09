#include "FirmwareMeasurements.h"
#include <Arduino.h>
#include <new>
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
    // Allocate the copy off the small async stack. Never hold the lock while
    // allocating, sampling heap, formatting JSON or touching a socket.
    auto copy=new(std::nothrow) MeasurementSnapshot;
    if(!copy) {request->send(503,"text/plain","measurement snapshot unavailable");return;}
    portENTER_CRITICAL(&mux);measurements.snapshot(*copy);portEXIT_CRITICAL(&mux);
    const auto current=resources(false);
    auto response=request->beginResponseStream("application/json");
    if(!response) {delete copy;request->send(503,"text/plain","measurement response unavailable");return;}
    response->addHeader("Cache-Control","no-store");
    response->printf("{\"schemaVersion\":1,\"uptimeMs\":%u,\"iterations\":%u,\"maxGapMs\":%u,\"overruns\":%u,\"sequence\":%u,\"overwritten\":%u,\"freeHeap\":%u,\"minHeap\":%u,\"largestBlock\":%u,",
      millis(),copy->iterations,copy->maxGapMs,copy->overruns,copy->sequence,copy->overwritten,current.freeHeap,current.minHeap,current.largestBlock);
    response->printf("\"encodeCalls\":%u,\"encodeUs\":%u,\"encodeMaxUs\":%u,\"writeCalls\":%u,\"writeUs\":%u,\"writeMaxUs\":%u,\"events\":[",copy->encodeCalls,copy->encodeUs,copy->encodeMaxUs,copy->writeCalls,copy->writeUs,copy->writeMaxUs);
    for(size_t i=0;i<copy->count;++i) {
      const auto &e=copy->events[i];
      response->printf("%s{\"sequence\":%u,\"atMs\":%u,\"phase\":\"%s\",\"end\":%s,\"elapsedUs\":%u,\"freeHeap\":%u,\"minHeap\":%u,\"largestBlock\":%u,\"stackBytes\":%u}",i?",":"",e.sequence,e.atMs,phaseName(e.phase),e.end?"true":"false",e.elapsedUs,e.resources.freeHeap,e.resources.minHeap,e.resources.largestBlock,e.resources.stackBytes);
    }
    response->print("]}");delete copy;request->send(response);
  });
}
}
