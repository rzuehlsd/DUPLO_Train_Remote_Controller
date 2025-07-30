#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Declare the mutex and initialization function
extern SemaphoreHandle_t serialMutex;
void SerialMUTEX();
void logWithMutex(const char *file, int line, const char *format, ...);

#ifdef DEBUG

#define DEBUG_LOG(format, ...) \
    logWithMutex(__FILE__, __LINE__, format, ##__VA_ARGS__);
#else
#define DEBUG_LOG(format, ...) ;
#endif

#endif // DEBUG_H