/*
 * SystemMemory.h
 * 
 * Provides functions to monitor and retrieve memory usage information for ESP32 devices.
 * 
 * This header file declares functions to print memory information and retrieve the amount of free
 * heap, SPIRAM, and flash memory available on the device. These utilities are useful for debugging
 * and monitoring system resource usage.
 * 
 * Author: Ralf Zühlsdorff
 * Date: 21. Juli 2025
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
