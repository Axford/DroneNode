/*
@type CloudTelemetry
@inherits Drone
@category Networking
@description Bounded selected local/peer observations over packed HTTPS without certificate validation
@pub 8;u32;1;w;reportInterval;Milliseconds between reports (1000-86400000, default 10000)
@pub 9;u32;3;r;requests;Attempted requests, fully admitted packets, failed or uncertain requests
@pub 10;u32;3;r;selection;Resolved clauses, pending clauses, dropped new addresses
@pub 11;u32;4;r;memory;Free heap, minimum heap, worker stack high-water bytes, largest free block
@pub 12;u32;3;r;timing;Maximum service gap ms, last report duration ms, last HTTP status
@pub 13;u32;3;r;flow;Captured publications, coalesced publications, tracked addresses
*/
#ifndef CLOUD_TELEMETRY_MODULE_H
#define CLOUD_TELEMETRY_MODULE_H
#include "../DroneModule.h"
#include "../CloudTelemetryConfig.h"
#include "../TelemetryReport.h"
struct CloudTelemetryWorker;
class CloudTelemetryModule:public DroneModule {
  telemetry::Config config;
  telemetry::Report report;
  CloudTelemetryWorker* worker=nullptr;
  uint32_t rate=10000,lastResolve=0,lastService=0,maxGap=0,lastDiagnostics=0;
  bool started=false,intervalChanged=false;
  static CloudTelemetryModule* instance;
  void resolve();
  bool ensureWorker();
public:
  CloudTelemetryModule(uint8_t id,DroneSystem* system);
  ~CloudTelemetryModule() override;
  bool isConfigurationString(const char* name) const override;
  telemetry::Setting configure(const char* name,const char* value) override;
  void configurationError() override;
  void saveExtraConfiguration(File* file) override;
  void setup() override;
  void loop() override;
  void handleLinkMessage(DroneLinkMsg* message) override;
  void onParamWrite(DRONE_PARAM_ENTRY* param) override;
  void enable() override;
  void disable() override;
  void waiting() override;
  void doShutdown() override;
};
#endif
