#include "BootRecoveryPolicy.h"
#include "QuarantineConfiguration.h"
#include <assert.h>
#include <string.h>

namespace {
using recovery::Phase;

void failStatus(recovery::BootRecord& boot) {
  boot.current.phase = Phase::ModuleSetup;
  boot.current.module = 240;
  strcpy(boot.current.type, "Status");
  boot.begin(true, false, 6, true);
}

void repeatFailure(recovery::BootRecord& boot, recovery::ConfigIdentity first, recovery::ConfigIdentity second) {
  boot.inspectConfig(first);
  failStatus(boot);
  assert(boot.mayRetry());
  boot.requestRetry(false);
  boot.begin(false, true, 3);
  boot.inspectConfig(second);
  failStatus(boot);
}

uint8_t approved(const char* source) {
  char config[512];
  strcpy(config, source);
  const auto result = recovery::approvedStatus(config);
  assert(strcmp(config, source) == 0);  // Hash the original bytes.
  return result;
}

void testQuarantine() {
  recovery::ConfigIdentity config;
  config.optionalStatus = 240;
  config.hash[0] = 123;
  recovery::BootRecord boot;
  repeatFailure(boot, config, config);
  boot.inspectConfig(config);
  assert(!boot.recovering && boot.quarantined == 240);
  boot.ready();
  boot.begin(true, false, 6, true);
  assert(boot.recovering && !boot.mayRetry() && !boot.quarantined);

  boot = recovery::BootRecord{};
  repeatFailure(boot, config, config);
  boot.inspectConfig(config);
  boot.requestRetry(true);
  boot.begin(false, true, 3);
  assert(!boot.recovering && !boot.quarantined && boot.failures == 0);

  auto changed = config;
  changed.hash[0] = 124;
  boot = recovery::BootRecord{};
  repeatFailure(boot, config, changed);
  boot.inspectConfig(changed);
  assert(boot.recovering && !boot.quarantined);
  boot = recovery::BootRecord{};
  repeatFailure(boot, config, config);
  boot.inspectConfig(changed);
  assert(boot.recovering && !boot.quarantined);
  boot = recovery::BootRecord{};
  repeatFailure(boot, {}, {});
  boot.inspectConfig({});
  assert(boot.recovering && !boot.quarantined);

  assert(approved("node=3\noptionalStatus=240\n[Status=240]\nname=Test\n") == 240);
  assert(!approved("node=3\n[Status=240]\nname=Test\n"));
  assert(!approved("node=3\noptionalStatus=240\n[Motor=240]\n"));
  assert(!approved("node=3\noptionalStatus=240\n[Status=240]\npublish=scene\n"));
  assert(!approved("node=3\noptionalStatus=240\n[Status=240]\n[Status=240]\n"));
  assert(!approved("node=3\noptionalStatus=240\n[Status=240]\nname=Test\n[Motor=4]\n$speed=@>Test.value1\n"));
  assert(!approved("node=3\noptionalStatus=240\n[Status=240]\n[Motor=4]\n$speed=3>240.8\n"));
}
}

int main() {
  testQuarantine();
  using recovery::Phase;
  recovery::BootRecord boot;
  boot.begin(false, false, 1);
  assert(!boot.recovering);
  boot.current.phase = Phase::ModuleSetup;
  boot.current.module = 12;
  strcpy(boot.current.type, "Motor");

  boot.begin(true, false, 6);
  assert(boot.recovering && boot.mayRetry());
  assert(boot.failureReason == 6 && boot.failure.phase == Phase::ModuleSetup);
  assert(boot.failure.module == 12 && strcmp(boot.failure.type, "Motor") == 0);
  boot.ready();
  boot.requestRetry(false);
  boot.begin(false, true, 3);
  assert(!boot.recovering && boot.failures == 1);
  boot.current.phase = Phase::Probation;
  boot.begin(true, false, 6);
  assert(boot.recovering && !boot.mayRetry());
  boot.ready();
  boot.begin(false, true, 3);
  assert(boot.recovering && !boot.mayRetry());
  boot.requestRetry(true);
  boot.begin(false, true, 3);
  assert(!boot.recovering);
  boot.ready();
  boot.begin(true, false, 6);
  assert(!boot.recovering && boot.failures == 0);

  boot.begin(true, false, 6);
  assert(boot.recovering);
  boot.ready();
  boot.begin(true, false, 6);
  assert(boot.recovering && !boot.mayRetry());
  assert(boot.failure.phase == Phase::Ready);
}
