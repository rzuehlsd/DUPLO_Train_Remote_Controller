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


#define TIMING  1 // Set to enable timing test case

// Forward declarations
void activateInstance();

// TrainController instance with DuploHub for BLE communication
DuploHub duploHub;

// State variables for train demo
bool demoRunning = false;
unsigned long lastDemoStep = 0;
int demoStep = 0;
const unsigned long DEMO_STEP_DURATION = 5000; // 1 second per step



// Callback function when hub connects
void onHubConnected() {
    DEBUG_LOG("TrainController: Hub connected - initializing train demo!");
    
    delay(1000); // Allow time for activation
    DEBUG_LOG("TrainController: Hub instance activated, starting demo sequence...");

    // Set hub name
    duploHub.setHubName("DuploTrainHub");
    DEBUG_LOG("TrainController: Connected to hub: %s", duploHub.getHubName().c_str());
    DEBUG_LOG("TrainController: Hub address: %s", duploHub.getHubAddress().c_str());
    
    // Start the demo
    demoRunning = true;
    demoStep = 0;
    lastDemoStep = millis();
    
    DEBUG_LOG("TrainController: Starting train demo sequence...");
}



// Callback function when hub disconnects
void onHubDisconnected() {
    DEBUG_LOG("TrainController: Hub disconnected - stopping all operations");
    
    // Stop the demo
    demoRunning = false;
    demoStep = 0;
    
    // Stop motor as safety measure (though hub is disconnected)
    duploHub.stopMotor();
    
    DEBUG_LOG("TrainController: Train demo stopped due to disconnection");
}






// Custom BLE task function
void bleTaskFunction(void *param) {
    while (true) {
        // Perform BLE operations
        duploHub.update();

        // Yield control to avoid watchdog resets
        vTaskDelay(100 / portTICK_PERIOD_MS); // Yield for 10ms
    }
}



void setup() {
    Serial.begin(115200);
    delay(5000);

    SerialMUTEX();

    DEBUG_LOG("TrainController (2 Core) : Starting up...");
    DEBUG_LOG("");
    printMemoryInfo();

    
    delay(1000); // Initialize DuploHub instance
    duploHub.init();
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
    DEBUG_LOG("TrainController: Ready - BLE task running, waiting for hub connection...");
} 



// Function to process the responseQueue and print detected color
void detectedColorCb(DuploEnums::DuploColor color) {
    DEBUG_LOG("TrainController:  Detected Color: %d", color);
    delay(200); // Allow time for color detection to take effect
    // duploHub.setLedColor(color);
}


// Function to process the responseQueue and print detected color
void detectedSpeedCb(int speed) {
    DEBUG_LOG("TrainController:  Detected Speed: %d", speed);
    delay(200); // Allow time for speed detection to take effect
    return;

    if (speed > 10)
    {
      DEBUG_LOG("Forward");
      duploHub.setMotorSpeed(speed/2);
      delay(100); // Allow time for motor speed to take effect
    }
    else if (speed < -10)
    {
      DEBUG_LOG("Back");
      duploHub.setMotorSpeed(speed/2);
      delay(100); // Allow time for motor speed to take effect
    }
    else
    {
      DEBUG_LOG("Stop");
      duploHub.stopMotor();
      delay(100); // Allow time for motor to stop
    }
    
}

void activateInstance() {

    static bool onece = false;
    if (onece) return; // Ensure this is only called once
    onece = true;
    delay(500);
    duploHub.listDevicePorts(); // List connected devices on the hub
    delay(1000);
    duploHub.activateRgbLight();
    delay(200);
    duploHub.activateBaseSpeaker();
    delay(200);
    duploHub.activateColorSensor();
    delay(200);
    duploHub.activateSpeedSensor();
    delay(200);

    // Add a public function to register the callback
    duploHub.setDetectedColorCallback(detectedColorCb);
    delay(200);
    duploHub.setDetectedSpeedCallback(detectedSpeedCb);
    delay(200);

}



void timingMotor(DuploHub& duploHub, bool& demoRunning, int& testCount) {
    if (duploHub.isConnected() && (testCount < 5)) {
        // Execute test case 1
        unsigned long currentMillis = millis();
        DEBUG_LOG("TrainController: setMotorSpeed called at: %lu", currentMillis);
        DEBUG_LOG("TrainController Demo: Motor forward (speed 35)");
        duploHub.setMotorSpeed(35);

        // Delay for 5 seconds
        delay(5000);

        testCount++;
    }
}

void timingSound(DuploHub& duploHub, bool& demoRunning, int& testCount) {
    if (duploHub.isConnected() && (testCount < 10)) {
        // Execute test case 1
        unsigned long currentMillis = millis();
        DEBUG_LOG("TrainController: playSound called at: %lu", currentMillis);
        DEBUG_LOG("TrainController Demo: Playing sound %d", 3 + testCount);
        duploHub.playSound((DuploEnums::DuploSound) 3 + testCount); // Cycle through sounds for testing

        // Delay for 5 seconds
        delay(5000);

        testCount = testCount + 2; // Increment by 2 to skip to the next sound
    }
}

void timingLED(DuploHub& duploHub, bool& demoRunning, int& testCount) {
    if (duploHub.isConnected() && demoRunning && testCount >= 0 && testCount <= 10) {
        // Execute test case 2
        unsigned long currentMillis = millis();
        DEBUG_LOG("TrainController: setLEDColor called at: %lu", millis());
        DEBUG_LOG("TrainController Demo: Setting LED to color %d", testCount);
        duploHub.setLedColor((DuploEnums::DuploColor)(testCount)); // Cycle through colors for testing

        // Delay for 5 seconds
        delay(5000);

        testCount++;
    }
}

void duploHubDemo(DuploHub& duploHub, bool& demoRunning, unsigned long& lastDemoStep, int& demoStep) {

  if (duploHub.isConnected() && demoRunning) {
        unsigned long currentTime = millis();

        // Check if it's time for the next demo step
        if (currentTime - lastDemoStep >= DEMO_STEP_DURATION) {
            lastDemoStep = currentTime;

            switch (demoStep) {
                case 0:
                    DEBUG_LOG("TrainController Demo: Setting LED to GREEN");
                    duploHub.setLedColor(DuploEnums::DuploColor::GREEN);
                    break;

                case 1:
                    DEBUG_LOG("TrainController Demo: Setting LED to RED");
                    duploHub.setLedColor(DuploEnums::DuploColor::RED);
                    break;

                case 2:
                    DEBUG_LOG("TrainController: setMotorSpeed called at: %lu", millis());
                    DEBUG_LOG("TrainController Demo: Motor forward (speed 35)");
                    duploHub.setMotorSpeed(35);
                    break;

                case 3:
                    DEBUG_LOG("TrainController Demo: Stop motor");
                    duploHub.stopMotor();
                    break;

                case 4:
                    DEBUG_LOG("TrainController Demo: Motor backward (speed -35)");
                    duploHub.setMotorSpeed(-35);
                    break;

                case 5:
                    DEBUG_LOG("TrainController Demo: Stop motor - demo complete");
                    duploHub.stopMotor();
                    duploHub.setLedColor(DuploEnums::DuploColor::GREEN); // Set to green to indicate completion
                    break;

                case 6:
                    DEBUG_LOG("TrainController Demo: Playing sound - HORN");
                    duploHub.playSound(DuploEnums::DuploSound::HORN);
                    break;

                case 7:
                    DEBUG_LOG("TrainController Demo: Playing sound - BELL");
                    duploHub.playSound(DuploEnums::DuploSound::BRAKE);
                    demoStep = -1; // Will be incremented to 0, restarting the demo
                    break;
            }

            demoStep++;
        }
    }
}

void checkMCUTemp() {
    static unsigned long lastTempCheck = 0;
    if (millis() - lastTempCheck >= 5000) { // Every 5 seconds
        lastTempCheck = millis();
        float cpuTemp = getCPUTemperature();
        DEBUG_LOG("CPU Temperature: %.2f °C", cpuTemp);
    }
}

void checkMCUStatus(DuploHub& duploHub, bool demoRunning) {
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 30000) { // Every 30 seconds
        DEBUG_LOG("TrainController Status - BLE Task: %s, Hub Connected: %s, Demo Active: %s",
                  duploHub.isBLETaskRunning() ? "Running" : "Stopped",
                  duploHub.isConnected() ? "Yes" : "No",
                  demoRunning ? "Yes" : "No");
        lastStatusUpdate = millis();
    }
}




// main loop
void loop() {

   // Handle connection callbacks (non-blocking)
  duploHub.update();

  // Process sensor callbacks frequently
  duploHub.processResponseQueue();

  if( duploHub.isConnected()) {
    activateInstance(); // Activate RGB light and speaker when connected
  }

duploHub.setMotorSpeed(35);
delay(100); // Shorter delay for more responsive sensor reading

duploHub.setMotorSpeed(0);
delay(100); 

duploHub.setLedColor(DuploEnums::DuploColor::RED);
delay(100); 

duploHub.setLedColor(DuploEnums::DuploColor::GREEN);
delay(100); 

duploHub.playSound(DuploEnums::DuploSound::HORN);
delay(1000); // Keep longer delay for sound to complete

duploHub.playSound(DuploEnums::DuploSound::BRAKE);
delay(1000); 




duploHub.processResponseQueue();
  

// Check CPU temperature every 5 seconds
checkMCUTemp();

delay(200); // Add a small delay to reduce CPU load
} // End of loop

