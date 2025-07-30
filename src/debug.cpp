#include "debug.h"
#include <stdio.h> // For snprintf
#include <stdarg.h>


// Define the mutex
SemaphoreHandle_t serialMutex = NULL;

// Initialize the mutex
void SerialMUTEX() {
    serialMutex = xSemaphoreCreateMutex();
    if (serialMutex == NULL) {
        Serial.println("ERROR: Failed to create serial mutex");
    }
}



void logWithMutex(const char* file, int line, const char* format, ...) {
    va_list args;
    va_start(args, format);
    char logMessage[256];
    vsnprintf(logMessage, sizeof(logMessage), format, args);
    va_end(args);
    if (serialMutex != NULL) {
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
            Serial.println(logMessage);
            Serial.flush();
            xSemaphoreGive(serialMutex);
        }
    } else {
        Serial.println(logMessage);
        Serial.flush();
    }
}
