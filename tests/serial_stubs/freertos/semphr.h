#pragma once
#include "FreeRTOS.h"
inline SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* value){return value;}
inline int xSemaphoreTake(SemaphoreHandle_t value,unsigned wait){if(wait){value->mutex.lock();return 1;}return value->mutex.try_lock();}
inline void xSemaphoreGive(SemaphoreHandle_t value){value->mutex.unlock();}
