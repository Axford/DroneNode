#include "BootRecoveryPolicy.h"
#include <assert.h>
#include <string.h>

int main() {
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
