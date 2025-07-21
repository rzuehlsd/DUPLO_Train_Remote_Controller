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
 * - Thread-safe command queuing for motor and LED control
 * 
 * (c) Copyright 2025
 * Released under MIT License
 * 
 */

#include "DuploHub.h"
#include "SystemMemory.h"

// TrainController instance with DuploHub for BLE communication
DuploHub duploHub;

// State variables for train demo
bool demoRunning = false;
unsigned long lastDemoStep = 0;
int demoStep = 0;
const unsigned long DEMO_STEP_DURATION = 1000; // 1 second per step

// Callback function when hub connects
void onHubConnected() {
    Serial.println("TrainController: Hub connected - initializing train demo!");
    
    // Set hub name
    duploHub.setHubName("DuploTrainHub");
    
    // Print hub information
    Serial.print("TrainController: Connected to hub: ");
    Serial.println(duploHub.getHubName().c_str());
    Serial.print("TrainController: Hub address: ");
    Serial.println(duploHub.getHubAddress().c_str());
    
    // Start the demo
    demoRunning = true;
    demoStep = 0;
    lastDemoStep = millis();
    
    Serial.println("TrainController: Starting train demo sequence...");
}

// Callback function when hub disconnects
void onHubDisconnected() {
    Serial.println("TrainController: Hub disconnected - stopping all operations");
    
    // Stop the demo
    demoRunning = false;
    demoStep = 0;
    
    // Stop motor as safety measure (though hub is disconnected)
    duploHub.stopMotor();
    
    Serial.println("TrainController: Train demo stopped due to disconnection");
}

// Custom BLE task function
void bleTaskFunction(void *param) {
    while (true) {
        // Perform BLE operations
        duploHub.update();

        // Yield control to avoid watchdog resets
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Yield for 10ms
    }
}

void setup() {
    Serial.begin(115200);
    delay(5000);

    Serial.println("TrainController (2 Core) : Starting up...");
    Serial.println();
    printMemoryInfo();

    delay(1000);

    // Register connection event callbacks
    duploHub.setOnConnectedCallback(onHubConnected);
    duploHub.setOnDisconnectedCallback(onHubDisconnected);

    // Start the BLE task for background connection management
    xTaskCreatePinnedToCore(
        bleTaskFunction,  // Task function
        "BLE Task",       // Task name
        4096,             // Stack size (increase if needed)
        NULL,             // Parameters
        1,                // Priority
        NULL,             // Task handle
        0                 // Core 0
    );

    delay(2000); // Allow time for BLE task to initialize
    Serial.println("TrainController: Ready - BLE task running, waiting for hub connection...");
} 


// main loop
void loop() {
  // Handle connection callbacks (non-blocking)
  duploHub.update();

  // Optional: Show system status periodically
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 30000) { // Every 30 seconds
    Serial.print("TrainController Status - BLE Task: ");
    Serial.print(duploHub.isBLETaskRunning() ? "Running" : "Stopped");
    Serial.print(", Hub Connected: ");
    Serial.print(duploHub.isConnected() ? "Yes" : "No");
    Serial.print(", Demo Active: ");
    Serial.println(demoRunning ? "Yes" : "No");
    lastStatusUpdate = millis();
  }

  // Run train demo sequence if connected and demo is active
  if (duploHub.isConnected() && demoRunning) {
    unsigned long currentTime = millis();

    // Check if it's time for the next demo step
    if (currentTime - lastDemoStep >= DEMO_STEP_DURATION) {
      lastDemoStep = currentTime;

      switch (demoStep) {
        case 0:
          Serial.println("TrainController Demo: Setting LED to GREEN");
          duploHub.setLedColor(GREEN);
          break;

        case 1:
          Serial.println("TrainController Demo: Setting LED to RED");
          duploHub.setLedColor(RED);
          break;

        case 2:
          Serial.println("TrainController Demo: Motor forward (speed 35)");
          duploHub.setMotorSpeed(35);
          break;

        case 3:
          Serial.println("TrainController Demo: Stop motor");
          duploHub.stopMotor();
          break;

        case 4:
          Serial.println("TrainController Demo: Motor backward (speed -35)");
          duploHub.setMotorSpeed(-35);
          break;

        case 5:
          Serial.println("TrainController Demo: Stop motor - demo complete");
          duploHub.stopMotor();
          duploHub.setLedColor(GREEN); // Set to green to indicate completion
          break;

        case 6:
          Serial.println("TrainController Demo: Playing sound - HORN");
          duploHub.playSound(HORN);
          break;

        case 7:
          Serial.println("TrainController Demo: Playing sound - BELL");
          duploHub.playSound(BRAKE);
          demoStep = -1; // Will be incremented to 0, restarting the demo
          break;
      }

      demoStep++;
    }
  }

  static unsigned long lastTempCheck = 0;
  if (millis() - lastTempCheck >= 5000) { // Every 5 seconds
    lastTempCheck = millis();
    float cpuTemp = getCPUTemperature();
    Serial.println("CPU Temperature: " + String(cpuTemp) + " °C");
  }


  vTaskDelay(100 / portTICK_PERIOD_MS); // Add a small delay to reduce CPU load
} // End of loop
