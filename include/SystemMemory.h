
/*
 * SystemMemory.h
 *
 * Description:
 *   Functions and utilities for monitoring and retrieving memory and resource usage on ESP32 devices.
 *   Includes heap, SPIRAM, flash, and CPU temperature queries for debugging and performance analysis.
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

#pragma once

#include "esp_heap_caps.h"
#include "esp_spi_flash.h"
#include "esp_system.h"

// Function to print memory information
void printMemoryInfo();

// Function to get free heap (RAM)
size_t getFreeHeap();

// Function to get free SPIRAM (if available)
size_t getFreeSPIRAM();

// Function to get free Flash (approximation)
size_t getFreeFlash();

// Function to get CPU temperature in Celsius
float getCPUTemperature();
