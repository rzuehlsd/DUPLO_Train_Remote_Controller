
/*
 * debug.cpp
 *
 * Description:
 *   Implementation of thread-safe debug logging for ESP32 LEGO DUPLO Train Controller.
 *   Provides mutex-protected serial output with file/line info for robust debugging in multi-core environments.
 *
 * Author: Ralf Zühlsdorff
 * Copyright (c) 2025 Ralf Zühlsdorff
 * License: MIT License
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
