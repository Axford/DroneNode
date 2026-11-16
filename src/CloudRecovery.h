#pragma once

#include <stdint.h>

namespace cloudRecovery {
void start(uint8_t node);
void service();
void usbCommand(const char* command, uint8_t node);
}
