/**
 * TrainController - A complete train control application using DuploHub class
 * Controls a DUPLO train with a motor connected to Port A of the Hub
 * 
 * Architecture:
 * - TrainController: Main application logic (this file)
 * - DuploHub: Hardware abstraction layer with multi-task BLE management
 * - Lpf2Hub: Low-level LEGO Powered Up protocol implementation
 * 
 * Features:
 * - Multi-task architecture with BLE operations in background task (Core 0)
 * - Non-blocking demo sequence running in main loop (Core 1)
 * - Automatic connection management and recovery
 * - Real-time status monitoring and logging
 * - Thread-safe command queuing for motor, LED, and sound control
 * - Sound tests integrated into the demo sequence
 * - Optimized BLE task responsiveness and reduced latency
 * - Improved sound playback timing
 * 
 * Demo Sequence:
 * - LED color changes (GREEN, RED)
 * - Motor control (forward, backward, stop)
 * - Sound playback (HORN, BELL)
 * 
 * (c) Copyright 2025
 * Released under MIT License
 */

#include "DuploHub.h"
#include "SystemMemory.h"


#define DEBUG 1 // Enable debug logging
#include "debug.h"


// TrainController instance with DuploHub for BLE communication
DuploHub duploHub;




// Function to process the responseQueue and print detected color
static void detectedColorCb(DuploEnums::DuploColor color) {
    DEBUG_LOG("TrainController:  Detected Color: %d", color);
    delay(200); // Allow time for color detection to take effect
    // duploHub.setLedColor(color);
}


// Function to process the responseQueue and print detected voltage
static void detectedVoltageCb(float voltage) {
    DEBUG_LOG("TrainController:  Detected Voltage: %.2f V", voltage);
    delay(200); // Allow time for voltage detection to take effect
}



// Function to process the responseQueue and print detected color
static void detectedSpeedCb(int speed) {
    DEBUG_LOG("TrainController:  Detected Speed: %d", speed);
    delay(200); // Allow time for speed detection to take effect
}



// Callback function when hub connects
static void onHubConnected() {
    DEBUG_LOG("TrainController: Hub instance activated, starting demo sequence...");

    #ifdef DEBUG
    duploHub.listDevicePorts(); // List connected devices on the hub
    delay(1000);
    #endif
    
    duploHub.activateRgbLight();
    delay(200);
    duploHub.activateBaseSpeaker();
    delay(200);
    duploHub.activateColorSensor();
    delay(200);
    duploHub.activateSpeedSensor();
    delay(200);
    duploHub.activateVoltageSensor();
    delay(200);
    DEBUG_LOG("DuploHub: All ports activated");
    
    duploHub.setDetectedColorCallback(detectedColorCb);
    delay(200);
    duploHub.setDetectedSpeedCallback(detectedSpeedCb);
    delay(200);
    duploHub.setDetectedVoltageCallback(detectedVoltageCb);
    delay(200);
    DEBUG_LOG("DuploHub: All callbacks registered");

    duploHub.setHubName("DuploTrain_1");
    DEBUG_LOG("TrainController: Connected to hub: %s", duploHub.getHubName().c_str());
    DEBUG_LOG("TrainController: Hub address: %s", duploHub.getHubAddress().c_str());
}



// Callback function when hub disconnects
static void onHubDisconnected() {
    DEBUG_LOG("TrainController: Hub disconnected - stopping all operations");
    
    // Stop motor as safety measure (though hub is disconnected)
    duploHub.stopMotor();
    
    DEBUG_LOG("TrainController: Train demo stopped due to disconnection");
}







void setup() {
    Serial.begin(115200);
    delay(5000);

    SerialMUTEX();
    DEBUG_LOG("TrainController (2 Core) : Starting up...");
    DEBUG_LOG("");

    printMemoryInfo();

    // Initialize DuploHub instance
    duploHub.init();
    delay(1000);

    // Register connection event callbacks
    duploHub.setOnConnectedCallback(onHubConnected);
    delay(200);
    duploHub.setOnDisconnectedCallback(onHubDisconnected);
    delay(200); // Allow time for BLE task to initialize

    DEBUG_LOG("TrainController: Ready - BLE task running, waiting for hub connection...");
} 





void checkStatus(DuploHub& duploHub) {
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 30000) { // Every 30 seconds
        DEBUG_LOG("TrainController Status - BLE Task: %s, Hub Connected: %s",
                  duploHub.isBLETaskRunning() ? "Running" : "Stopped",
                  duploHub.isConnected() ? "Yes" : "No");
        float cpuTemp = getCPUTemperature();
        DEBUG_LOG("CPU Temperature: %.2f °C", cpuTemp);
        lastStatusUpdate = millis();
    }
}




// main loop
void loop() {

    static int color = 0; // Color index for LED cycling
    static int sound = 3; // Sound index for sound playback
    static int dir = 1; // Direction for motor control

    if(duploHub.isConnected()) {
        
        // do something
        duploHub.setLedColor((DuploEnums::DuploColor)color);
        delay(1000);
        color = (color + 1) % 11; // Cycle through colors 0-10

        // Play sound
        duploHub.playSound(DuploEnums::DuploSound::HORN);
        delay(1000);
        sound = (sound + 2) % 6;   // Cycle through sounds 0-4

        // Set motor speed
        duploHub.setMotorSpeed(dir *35);
        delay(2000);
        duploHub.stopMotor();
        dir = -dir; // Reverse direction for next cycle

        duploHub.processResponseQueue();
    }

    // Check Duplo Hub State
    checkStatus(duploHub);

    delay(1000); // Add a small delay to reduce CPU load
} // End of loop

