/**
 * TrainController_Extended - Example using bidirectional sensor data architecture
 * 
 * This example demonstrates:
 * - Color sensor integration with callback system
 * - Distance sensor monitoring
 * - Button press handling
 * - Speed control based on detected colors
 * - All running in multi-task architecture with thread-safe communication
 * 
 * Hardware Setup:
 * - DUPLO Train Hub
 * - Motor on Port A
 * - Color/Distance sensor on Port B
 * - Built-in hub button
 * 
 * (c) Copyright 2025
 * Released under MIT License
 * 
 */

#include "DuploHubExtended.h"

// Extended TrainController instance with sensor support
DuploHubExtended duploHub;

// State variables
bool demoRunning = false;
bool sensorsActive = false;
int lastDetectedColor = -1;

// ============================================================================
// CONNECTION CALLBACKS
// ============================================================================

void onHubConnected() {
    Serial.println("TrainController Extended: Hub connected!");
    
    duploHub.setHubName("DuploTrainSensor");
    duploHub.setLedColor(GREEN);
    
    Serial.println("TrainController Extended: Activating sensors...");
    
    // Activate sensors (commands will be queued to BLE task)
    duploHub.activateColorSensor((byte)PoweredUpHubPort::B);
    duploHub.activateDistanceSensor((byte)PoweredUpHubPort::B);  // Same port, both color and distance
    duploHub.activateButton();
    
    sensorsActive = true;
    Serial.println("TrainController Extended: Sensors activation commands sent");
}

void onHubDisconnected() {
    Serial.println("TrainController Extended: Hub disconnected");
    
    demoRunning = false;
    sensorsActive = false;
    duploHub.stopMotor();
}

// ============================================================================
// SENSOR CALLBACKS (Called in Main Task Context)
// ============================================================================

void onColorDetected(int color, byte port) {
    Serial.print("TrainController Extended: Color detected - ");
    Serial.print(LegoinoCommon::ColorStringFromColor(color).c_str());
    Serial.print(" on port ");
    Serial.println(port);
    
    lastDetectedColor = color;
    
    // Set LED to detected color
    duploHub.setLedColor((Color)color);
    
    // Control train based on color (similar to original TrainColor example)
    if (color == (byte)Color::RED) {
        Serial.println("TrainController Extended: RED detected - STOP");
        duploHub.stopMotor();
    } 
    else if (color == (byte)Color::YELLOW) {
        Serial.println("TrainController Extended: YELLOW detected - SLOW (speed 25)");
        duploHub.setMotorSpeed(25);
    }
    else if (color == (byte)Color::BLUE) {
        Serial.println("TrainController Extended: BLUE detected - MEDIUM (speed 35)");
        duploHub.setMotorSpeed(35);
    }
    else if (color == (byte)Color::GREEN) {
        Serial.println("TrainController Extended: GREEN detected - FAST (speed 50)");
        duploHub.setMotorSpeed(50);
    }
    else {
        Serial.println("TrainController Extended: Other color - SLOW (speed 15)");
        duploHub.setMotorSpeed(15);
    }
}

void onDistanceDetected(int distance, byte port) {
    Serial.print("TrainController Extended: Distance detected - ");
    Serial.print(distance);
    Serial.print(" cm on port ");
    Serial.println(port);
    
    // Emergency stop if object too close
    if (distance < 5 && distance > 0) {  // 0 can be error value
        Serial.println("TrainController Extended: EMERGENCY STOP - Object too close!");
        duploHub.stopMotor();
        duploHub.setLedColor(RED);
    }
}

void onButtonPressed(ButtonState state) {
    Serial.print("TrainController Extended: Button ");
    
    if (state == ButtonState::PRESSED) {
        Serial.println("PRESSED - Starting/Restarting train");
        
        // Start train at medium speed if stopped
        duploHub.setMotorSpeed(25);
        duploHub.setLedColor(BLUE);
        demoRunning = true;
    }
    else if (state == ButtonState::RELEASED) {
        Serial.println("RELEASED");
    }
}

// ============================================================================
// MAIN APPLICATION
// ============================================================================

void setup() {
    Serial.begin(115200);
    
    Serial.println("TrainController Extended: Starting with sensor support...");
    
    // Register connection callbacks
    duploHub.setOnConnectedCallback(onHubConnected);
    duploHub.setOnDisconnectedCallback(onHubDisconnected);
    
    // Register sensor callbacks
    duploHub.setOnColorSensorCallback(onColorDetected);
    duploHub.setOnDistanceSensorCallback(onDistanceDetected);
    duploHub.setOnButtonCallback(onButtonPressed);
    
    // Start multi-task BLE system
    duploHub.startBLETask();
    
    Serial.println("TrainController Extended: Ready - waiting for hub connection...");
}

void loop() {
    // Handle ALL callbacks and sensor data (non-blocking)
    duploHub.update();
    
    // Optional: System status monitoring
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 15000) { // Every 15 seconds
        Serial.println("=== TrainController Extended Status ===");
        Serial.print("BLE Task Running: ");
        Serial.println(duploHub.isBLETaskRunning() ? "Yes" : "No");
        Serial.print("Hub Connected: ");
        Serial.println(duploHub.isConnected() ? "Yes" : "No");
        Serial.print("Sensors Active: ");
        Serial.println(sensorsActive ? "Yes" : "No");
        Serial.print("Last Color: ");
        if (lastDetectedColor >= 0) {
            Serial.println(LegoinoCommon::ColorStringFromColor(lastDetectedColor).c_str());
        } else {
            Serial.println("None");
        }
        Serial.println("=====================================");
        lastStatusUpdate = millis();
    }
    
    // Small delay to prevent tight loop
    delay(10);
}

// ============================================================================
// ARCHITECTURE SUMMARY
// ============================================================================

/*

BIDIRECTIONAL DATA FLOW:

┌─────────────────────────────────────────────────────────────────────────┐
│                         EXTENDED ARCHITECTURE                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  MAIN TASK (Core 1)           QUEUES              BLE TASK (Core 0)     │
│  ┌─────────────────┐                             ┌─────────────────┐    │
│  │ TrainController │                             │   BLE Manager   │    │
│  │                 │                             │                 │    │
│  │ • Motor Control │ ────── commandQueue ──────► │ • Lpf2Hub calls │    │
│  │ • Sensor Setup  │                             │ • Sensor setup  │    │
│  │ • User Callbacks│                             │ • BLE operations│    │
│  │                 │ ◄────── sensorQueue ──────  │ • Sensor data   │    │
│  │ • Status Display│                             │ • Data queuing  │    │
│  └─────────────────┘                             └─────────────────┘    │
│                                                                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  COMMAND FLOW (Main → BLE):                                             │
│  1. activateColorSensor() → CMD_ACTIVATE_COLOR_SENSOR → commandQueue    │
│  2. BLE Task: hub.activatePortDevice(port, callbackWrapper)             │
│  3. setMotorSpeed() → CMD_MOTOR_SPEED → commandQueue                    │
│  4. BLE Task: hub.setBasicMotorSpeed(port, speed)                       │
│                                                                         │
│  SENSOR FLOW (BLE → Main):                                              │
│  1. DUPLO sensor → Lpf2Hub callback → callbackWrapper                   │
│  2. BLE Task: Create SensorData → sensorQueue                           │
│  3. Main Task: processSensorData() → User callback                      │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘

BENEFITS:
✅ Non-blocking sensor processing
✅ Thread-safe sensor activation
✅ Real-time color-based speed control
✅ Emergency distance detection
✅ Button-based manual control
✅ Professional error handling
✅ Clean callback-based architecture

*/
