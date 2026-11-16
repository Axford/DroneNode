#pragma once
#include <mutex>
#define pdTRUE 1
#define portMAX_DELAY 0xffffffff
struct StaticSemaphore_t {std::mutex mutex;};
using SemaphoreHandle_t=StaticSemaphore_t*;
