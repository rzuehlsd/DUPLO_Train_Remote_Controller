/**
 * @file debug.h
 * @brief Thread-safe debug logging helpers for the ESP32 DUPLO Train controller.
 *
 * Provides mutex-protected logging macros that prepend file and line metadata to each
 * message so concurrent FreeRTOS tasks can write to the serial console without overlapping
 * output. All declarations in this header are implemented in `src/debug.cpp`.
 *
 * @author Ralf Zühlsdorff
 * @date 2025
 *
 * @copyright
 * MIT License
 */
#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Declare the mutex and initialization function
extern SemaphoreHandle_t serialMutex;

/**
 * @brief Initialize the mutex that guards serial logging.
 */
void SerialMUTEX();

/**
 * @brief Print a formatted message guarded by the serial mutex.
 *
 * @param file Name of the translation unit emitting the log entry.
 * @param line Source line for the log entry.
 * @param format `printf`-style format string for the payload.
 * @param ... Values that satisfy the `format` placeholders.
 */
void logWithMutex(const char *file, int line, const char *format, ...);

#ifdef DEBUG

#define DEBUG_LOG(format, ...) \
    logWithMutex(__FILE__, __LINE__, format, ##__VA_ARGS__);
#else
#define DEBUG_LOG(format, ...) ;
#endif

#endif // DEBUG_H