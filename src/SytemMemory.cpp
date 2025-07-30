/*
 * SystemMemory.cpp
 * 
 * Implements functions to monitor and retrieve memory usage information for ESP32 devices.
 * 
 * This source file provides the implementation of functions declared in SystemMemory.h. These
 * functions include utilities to print memory information and retrieve the amount of free heap,
 * SPIRAM, and flash memory available on the device. These utilities are useful for debugging and
 * monitoring system resource usage. Additionally, it includes functionality to monitor CPU
 * temperature in real-time.
 * 
 * Author: Ralf Zühlsdorff
 * Date: 21. Juli 2025
 */

#include <Arduino.h>
#include "SystemMemory.h"



/**
 * @brief Calculate used flash size based on partition table.
 * @return Used flash size in bytes.
 *
 * Finds the application partition and returns the end address as used flash size.
 */
uint32_t spi_flash_get_used_size() {
    const esp_partition_t* partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, 
                                                    ESP_PARTITION_SUBTYPE_ANY, NULL);
    if (partition == NULL) {
        Serial.println("ERROR: Unable to find application partition");
        return 0;
    }
    // Calculate used flash size as the end address of the application partition
    return partition->address + partition->size;
}


/**
 * @brief Print memory information to the serial console.
 *
 * Prints free RAM, SPIRAM support, flash size, free flash, and CPU temperature.
 */
void printMemoryInfo() {
    Serial.println("_______________________________________________");
    Serial.println("               Memory Information:");
    Serial.println();
    // Print free heap (RAM)
    size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    Serial.println("Free RAM: " + String(freeHeap) + " bytes");
    if (freeHeap > 512*1024) // Check if free RAM is more than 512KB
        Serial.println("SPIRAM is supported");
    // Print flash size
    uint32_t flashSize = spi_flash_get_chip_size();
    Serial.println("Flash Size: " + String(flashSize) + " bytes");
    // Print free flash (approximation)
    uint32_t freeFlash = spi_flash_get_chip_size() - spi_flash_get_used_size();
    Serial.println("Free Flash: " + String(freeFlash) + " bytes");
    // Print CPU temperature
    float cpuTemp = getCPUTemperature();
    Serial.println("CPU Temperature: " + String(cpuTemp) + " °C");
    Serial.println();
    Serial.println("_______________________________________________");
}


/**
 * @brief Get free heap (RAM) in bytes.
 * @return Free heap size in bytes.
 */
size_t getFreeHeap() {
    return heap_caps_get_free_size(MALLOC_CAP_8BIT);
}


/**
 * @brief Get free SPIRAM (if available).
 * @return Free SPIRAM size in bytes, or 0 if not supported.
 */
size_t getFreeSPIRAM() {
#if CONFIG_SPIRAM_SUPPORT
    return heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
#else
    return 0; // SPIRAM not supported
#endif
}


/**
 * @brief Get free Flash size (approximation).
 * @return Free flash size in bytes.
 */
size_t getFreeFlash() {
    uint32_t flashSize = spi_flash_get_chip_size();
    uint32_t usedFlash = spi_flash_get_used_size();
    return flashSize - usedFlash;
}


/**
 * @brief Get CPU temperature in Celsius.
 * @return CPU temperature as float.
 *
 * Uses Arduino's built-in function to read CPU temperature.
 */
float getCPUTemperature() {
    // Use Arduino's built-in function to read CPU temperature
    return temperatureRead();
}


