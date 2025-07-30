#ifndef DEBUG_H
#define DEBUG_H

/*
 * debug.h
 *
 * Description:
 *   Thread-safe debug logging macros and functions for ESP32 LEGO DUPLO Train Controller.
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
#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Declare the mutex and initialization function
extern SemaphoreHandle_t serialMutex;

// Call SerialMUTEX() in setup() to initialize the mutex
// This function should be called once in the setup() function prior to any use of DEBUG_LOG
// This ensures that all debug logs are printed in a thread-safe manner
// If SerialMUTEX() is not called, DEBUG_LOG will still work but without mutex protection
void SerialMUTEX();
void logWithMutex(const char *file, int line, const char *format, ...);

#ifdef DEBUG

#define DEBUG_LOG(format, ...) \
    logWithMutex(__FILE__, __LINE__, format, ##__VA_ARGS__);
#else
#define DEBUG_LOG(format, ...) ;
#endif

#endif // DEBUG_H