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
    if (serialMutex != NULL) {
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
            // Combine file, line, and format into a single formatted string
            char extendedFormat[256];
            snprintf(extendedFormat, sizeof(extendedFormat), "[%s:%d] %s", file, line, format);

            char logMessage[512];
            va_list args;
            va_start(args, format);
            vsnprintf(logMessage, sizeof(logMessage), extendedFormat, args); // Use vsnprintf to format the message
            va_end(args);

            Serial.println(logMessage); // Print the formatted message
            Serial.flush();
            xSemaphoreGive(serialMutex);
        }
    }
}
