/**
 * A refactored example using DuploHub class to control a train
 * which has a motor connected to the Port A of the Hub
 * 
 * (c) Copyright 2025
 * Released under MIT License
 * 
 */

#include "DuploHub.h"

// create a DuploHub instance
DuploHub duploHub;

// State variables for train demo
bool demoRunning = false;
unsigned long lastDemoStep = 0;
int demoStep = 0;
const unsigned long DEMO_STEP_DURATION = 1000; // 1 second per step

// Callback function when hub connects
void onHubConnected() {
    Serial.println("Callback: Hub connected - initializing train demo!");
    
    // Set hub name
    duploHub.setHubName("DuploTrainHub");
    
    // Print hub information
    Serial.print("Connected to hub: ");
    Serial.println(duploHub.getHubName().c_str());
    Serial.print("Hub address: ");
    Serial.println(duploHub.getHubAddress().c_str());
    
    // Start the demo
    demoRunning = true;
    demoStep = 0;
    lastDemoStep = millis();
    
    Serial.println("Starting train demo sequence...");
}

// Callback function when hub disconnects
void onHubDisconnected() {
    Serial.println("Callback: Hub disconnected - stopping all operations");
    
    // Stop the demo
    demoRunning = false;
    demoStep = 0;
    
    // Stop motor as safety measure (though hub is disconnected)
    duploHub.stopMotor();
    
    Serial.println("Train demo stopped due to disconnection");
}

void setup() {
    Serial.begin(115200);
    
    // Register callbacks
    duploHub.setOnConnectedCallback(onHubConnected);
    duploHub.setOnDisconnectedCallback(onHubDisconnected);
    
    Serial.println("Train controller started, waiting for hub connection...");
} 


// main loop
void loop() {
  // Handle connection and basic hub management
  duploHub.update();

  // Run train demo sequence if connected and demo is active
  if (duploHub.isConnected() && demoRunning) {
    unsigned long currentTime = millis();
    
    // Check if it's time for the next demo step
    if (currentTime - lastDemoStep >= DEMO_STEP_DURATION) {
      lastDemoStep = currentTime;
      
      switch (demoStep) {
        case 0:
          Serial.println("Demo: Setting LED to GREEN");
          duploHub.setLedColor(GREEN);
          break;
          
        case 1:
          Serial.println("Demo: Setting LED to RED");
          duploHub.setLedColor(RED);
          break;
          
        case 2:
          Serial.println("Demo: Motor forward (speed 35)");
          duploHub.setMotorSpeed(35);
          break;
          
        case 3:
          Serial.println("Demo: Stop motor");
          duploHub.stopMotor();
          break;
          
        case 4:
          Serial.println("Demo: Motor backward (speed -35)");
          duploHub.setMotorSpeed(-35);
          break;
          
        case 5:
          Serial.println("Demo: Stop motor - demo complete");
          duploHub.stopMotor();
          duploHub.setLedColor(GREEN); // Set to green to indicate completion
          demoStep = -1; // Will be incremented to 0, restarting the demo
          break;
      }
      
      demoStep++;
    }
  }
  
} // End of loop
