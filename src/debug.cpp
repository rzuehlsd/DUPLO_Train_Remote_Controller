/**
 * @file debug.cpp
 * @brief Implementation of mutex-protected serial logging utilities.
 *
 * Provides the definitions for the helpers declared in `include/debug.h`, enabling formatted
 * debug output that stays readable even when emitted from multiple FreeRTOS tasks.
 *
 * @author Ralf Zühlsdorff
 * @date 2025
 *
 * @copyright
 * MIT License
 */
#include "debug.h"
#include <stdio.h> // For snprintf
#include <stdarg.h>

// Define the mutex
SemaphoreHandle_t serialMutex = NULL;


/**
 * @brief Initialize the serial mutex for thread-safe logging.
 *
 * Call this function once in setup() before using DEBUG_LOG macros.
 * Ensures all debug output is protected by a FreeRTOS mutex.
 * If not called, DEBUG_LOG will still work but without mutex protection.
 */
void SerialMUTEX()
{
    serialMutex = xSemaphoreCreateMutex();
    if (serialMutex == NULL)
    {
        Serial.println("ERROR: Failed to create serial mutex");
    }
}


/**
 * @brief Print a formatted debug message with file and line info, thread-safe.
 *
 * @param file Source file name (use __FILE__ macro)
 * @param line Source line number (use __LINE__ macro)
 * @param format printf-style format string
 * @param ... Arguments for format string
 *
 * If the mutex is initialized, output is protected; otherwise, prints directly.
 */
void logWithMutex(const char *file, int line, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    char userMessage[192];
    vsnprintf(userMessage, sizeof(userMessage), format, args);
    va_end(args);
    char logMessage[256];
    snprintf(logMessage, sizeof(logMessage), "%s:%d: %s", file, line, userMessage);
    if (serialMutex != NULL)
    {
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE)
        {
            Serial.println(logMessage);
            Serial.flush();
            xSemaphoreGive(serialMutex);
        }
    }
    else
    {
        Serial.println(logMessage);
        Serial.flush();
    }
}
