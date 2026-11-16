#pragma once

#include "BootRecoveryPolicy.h"
#include <FS.h>

struct RecoveryProfile {
  uint16_t port = 8007;
  uint8_t node = 0;
  uint8_t channel = 2;
  uint8_t broadcast[4] = {255, 255, 255, 255};
  uint8_t server[4] = {};
  bool valid() const { return node > 0 && node < 255 && channel > 1 && channel < 255 && port; }
};

namespace bootRecovery {
void begin();
bool active();
void enter(recovery::Phase phase, uint8_t module = 0, const char* type = "");
void requestRestart();
void forceRecovery();
void deferAutomaticRetry();
void readCandidate(fs::FS& fs);
const RecoveryProfile& profile();
void completedPass(uint32_t now);
const recovery::Breadcrumb& failure();
uint8_t failureReason();
}
