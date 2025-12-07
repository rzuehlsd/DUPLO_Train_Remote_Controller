/**
 * @file SystemMemory.h
 * @brief Utilities for capturing ESP32 memory and temperature diagnostics.
 *
 * Declares helper routines that expose heap, SPIRAM, flash usage, and CPU temperature to aid in 
 * runtime monitoring and debugging of the DUPLO Train firmware.
 *
 * @author Ralf Zühlsdorff
 * @date 2025
 *
 * @copyright
 * MIT License
 */

#pragma once

#include "esp_heap_caps.h"
#include "esp_spi_flash.h"
#include "esp_system.h"

/** @brief Print a formatted memory and CPU usage report. */
void printMemoryInfo();

/** @brief Return the amount of free heap memory in bytes. */
size_t getFreeHeap();

/** @brief Return the amount of free SPIRAM in bytes, or zero when unavailable. */
size_t getFreeSPIRAM();

/** @brief Estimate the remaining flash capacity in bytes. */
size_t getFreeFlash();

/** @brief Report the current CPU temperature in degrees Celsius. */
float getCPUTemperature();
