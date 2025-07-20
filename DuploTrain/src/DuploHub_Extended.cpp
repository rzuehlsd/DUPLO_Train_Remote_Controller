/**
 * DuploHub_Extended - Implementation with bidirectional sensor data support
 * 
 * Key Implementation Points:
 * 1. Second FreeRTOS queue for sensor data (BLE → Main)
 * 2. Static callback wrappers to integrate with Lpf2Hub
 * 3. Sensor data processing in main update loop
 * 4. Thread-safe sensor activation commands
 * 
 * (c) Copyright 2025
 * Released under MIT License
 * 
 */

#include "Arduino.h"
#include "DuploHub_Extended.h"

// Global pointer for static callback access (alternative: use user data pointer)
static DuploHubExtended* g_duploHubInstance = nullptr;

// Default constructor
DuploHubExtended::DuploHubExtended() : motorPort((byte)PoweredUpHubPort::A), wasConnected(false),
                       connectionState(false), connectingState(false),
                       connectionMutex(nullptr), commandQueue(nullptr), sensorQueue(nullptr), 
                       bleTaskHandle(nullptr), onConnectedCallback(nullptr), 
                       onDisconnectedCallback(nullptr), onColorSensorCallback(nullptr),
                       onDistanceSensorCallback(nullptr), onButtonCallback(nullptr) {
    g_duploHubInstance = this;  // Set global instance for static callbacks
    initFreeRTOS();
}

// Initialize FreeRTOS objects
void DuploHubExtended::initFreeRTOS() {
    // Create mutex for thread-safe access to connection state
    connectionMutex = xSemaphoreCreateMutex();
    if (connectionMutex == nullptr) {
        Serial.println("ERROR: Failed to create connection mutex");
    }
    
    // Create command queue for Main → BLE communication
    commandQueue = xQueueCreate(10, sizeof(HubCommand));  // Queue size: 10 commands
    if (commandQueue == nullptr) {
        Serial.println("ERROR: Failed to create command queue");
    }
    
    // Create sensor queue for BLE → Main communication  
    sensorQueue = xQueueCreate(20, sizeof(SensorData));   // Queue size: 20 sensor readings
    if (sensorQueue == nullptr) {
        Serial.println("ERROR: Failed to create sensor queue");
    }
    
    Serial.println("DuploHub Extended: FreeRTOS objects initialized (bidirectional queues)");
}

// ============================================================================
// SENSOR ACTIVATION (Main Task → BLE Task via Command Queue)
// ============================================================================

void DuploHubExtended::activateColorSensor(byte port) {
    activateColorSensor_ThreadSafe(port);
}

void DuploHubExtended::activateColorSensor_ThreadSafe(byte port) {
    if (commandQueue != nullptr) {
        HubCommand cmd;
        cmd.type = CMD_ACTIVATE_COLOR_SENSOR;
        cmd.data.sensor.port = port;
        
        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            Serial.println("WARNING: Failed to queue color sensor activation command");
        }
    }
}

void DuploHubExtended::activateDistanceSensor(byte port) {
    activateDistanceSensor_ThreadSafe(port);
}

void DuploHubExtended::activateDistanceSensor_ThreadSafe(byte port) {
    if (commandQueue != nullptr) {
        HubCommand cmd;
        cmd.type = CMD_ACTIVATE_DISTANCE_SENSOR;
        cmd.data.sensor.port = port;
        
        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            Serial.println("WARNING: Failed to queue distance sensor activation command");
        }
    }
}

void DuploHubExtended::activateButton() {
    activateButton_ThreadSafe();
}

void DuploHubExtended::activateButton_ThreadSafe() {
    if (commandQueue != nullptr) {
        HubCommand cmd;
        cmd.type = CMD_ACTIVATE_BUTTON;
        
        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            Serial.println("WARNING: Failed to queue button activation command");
        }
    }
}

// ============================================================================
// COMMAND QUEUE PROCESSING (BLE Task - Extended with Sensor Activation)
// ============================================================================

void DuploHubExtended::processCommandQueue() {
    if (commandQueue == nullptr) return;
    
    HubCommand cmd;
    
    // Process all available commands (non-blocking)
    while (xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
        // Only process commands if connected (except sensor activation)
        if (!hub.isConnected() && cmd.type < CMD_ACTIVATE_COLOR_SENSOR) {
            Serial.println("BLE Task: Skipping command - hub not connected");
            continue;
        }
        
        switch (cmd.type) {
            case CMD_MOTOR_SPEED:
                Serial.print("BLE Task: Setting motor speed to ");
                Serial.println(cmd.data.motor.speed);
                hub.setBasicMotorSpeed(motorPort, cmd.data.motor.speed);
                break;
                
            case CMD_STOP_MOTOR:
                Serial.println("BLE Task: Stopping motor");
                hub.stopBasicMotor(motorPort);
                break;
                
            case CMD_SET_LED_COLOR:
                Serial.print("BLE Task: Setting LED color to ");
                Serial.println(cmd.data.led.color);
                hub.setLedColor(cmd.data.led.color);
                break;
                
            case CMD_SET_HUB_NAME:
                Serial.print("BLE Task: Setting hub name to ");
                Serial.println(cmd.data.hubName.name);
                hub.setHubName(cmd.data.hubName.name);
                break;
                
            // NEW: Sensor activation commands
            case CMD_ACTIVATE_COLOR_SENSOR:
                if (hub.isConnected()) {
                    Serial.print("BLE Task: Activating color sensor on port ");
                    Serial.println(cmd.data.sensor.port);
                    hub.activatePortDevice(cmd.data.sensor.port, colorSensorCallbackWrapper);
                }
                break;
                
            case CMD_ACTIVATE_DISTANCE_SENSOR:
                if (hub.isConnected()) {
                    Serial.print("BLE Task: Activating distance sensor on port ");
                    Serial.println(cmd.data.sensor.port);
                    hub.activatePortDevice(cmd.data.sensor.port, distanceSensorCallbackWrapper);
                }
                break;
                
            case CMD_ACTIVATE_BUTTON:
                if (hub.isConnected()) {
                    Serial.println("BLE Task: Activating button sensor");
                    hub.activateHubPropertyUpdate(HubPropertyReference::BUTTON, buttonCallbackWrapper);
                }
                break;
                
            default:
                Serial.println("BLE Task: Unknown command type");
                break;
        }
    }
}

// ============================================================================
// STATIC CALLBACK WRAPPERS (Called by Lpf2Hub in BLE Task Context)
// ============================================================================

void DuploHubExtended::colorSensorCallbackWrapper(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData) {
    if (g_duploHubInstance == nullptr) return;
    
    Lpf2Hub *lpf2Hub = (Lpf2Hub *)hub;
    
    if (deviceType == DeviceType::COLOR_DISTANCE_SENSOR) {
        int color = lpf2Hub->parseColor(pData);
        
        // Create sensor data structure
        SensorData sensorData;
        sensorData.type = SENSOR_COLOR;
        sensorData.port = portNumber;
        sensorData.timestamp = millis();
        sensorData.data.colorSensor.color = color;
        
        // Queue sensor data to main task
        g_duploHubInstance->queueSensorData(sensorData);
        
        Serial.print("BLE Task: Color sensor data queued - Color: ");
        Serial.print(LegoinoCommon::ColorStringFromColor(color).c_str());
        Serial.print(", Port: ");
        Serial.println(portNumber);
    }
}

void DuploHubExtended::distanceSensorCallbackWrapper(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData) {
    if (g_duploHubInstance == nullptr) return;
    
    Lpf2Hub *lpf2Hub = (Lpf2Hub *)hub;
    
    if (deviceType == DeviceType::COLOR_DISTANCE_SENSOR) {
        int distance = lpf2Hub->parseDistance(pData);
        
        // Create sensor data structure
        SensorData sensorData;
        sensorData.type = SENSOR_DISTANCE;
        sensorData.port = portNumber;
        sensorData.timestamp = millis();
        sensorData.data.distanceSensor.distance = distance;
        
        // Queue sensor data to main task
        g_duploHubInstance->queueSensorData(sensorData);
        
        Serial.print("BLE Task: Distance sensor data queued - Distance: ");
        Serial.print(distance);
        Serial.print(", Port: ");
        Serial.println(portNumber);
    }
}

void DuploHubExtended::buttonCallbackWrapper(void *hub, HubPropertyReference hubProperty, uint8_t *pData) {
    if (g_duploHubInstance == nullptr) return;
    
    Lpf2Hub *lpf2Hub = (Lpf2Hub *)hub;
    
    if (hubProperty == HubPropertyReference::BUTTON) {
        ButtonState buttonState = lpf2Hub->parseHubButton(pData);
        
        // Create sensor data structure
        SensorData sensorData;
        sensorData.type = SENSOR_BUTTON;
        sensorData.port = 0;  // Button is not port-specific
        sensorData.timestamp = millis();
        sensorData.data.button.state = buttonState;
        
        // Queue sensor data to main task
        g_duploHubInstance->queueSensorData(sensorData);
        
        Serial.print("BLE Task: Button data queued - State: ");
        Serial.println((byte)buttonState, HEX);
    }
}

// ============================================================================
// SENSOR DATA QUEUING (BLE Task → Main Task)
// ============================================================================

void DuploHubExtended::queueSensorData(const SensorData& data) {
    if (sensorQueue != nullptr) {
        if (xQueueSend(sensorQueue, &data, 0) != pdTRUE) {
            Serial.println("WARNING: Sensor data queue full - dropping data");
        }
    }
}

// ============================================================================
// SENSOR DATA PROCESSING (Main Task - Called from update())
// ============================================================================

void DuploHubExtended::processSensorData() {
    if (sensorQueue == nullptr) return;
    
    SensorData data;
    
    // Process all available sensor data (non-blocking)
    while (xQueueReceive(sensorQueue, &data, 0) == pdTRUE) {
        switch (data.type) {
            case SENSOR_COLOR:
                Serial.print("Main Task: Color sensor callback - Color: ");
                Serial.print(LegoinoCommon::ColorStringFromColor(data.data.colorSensor.color).c_str());
                Serial.print(", Port: ");
                Serial.println(data.port);
                
                // Call user callback if registered
                if (onColorSensorCallback != nullptr) {
                    onColorSensorCallback(data.data.colorSensor.color, data.port);
                }
                break;
                
            case SENSOR_DISTANCE:
                Serial.print("Main Task: Distance sensor callback - Distance: ");
                Serial.print(data.data.distanceSensor.distance);
                Serial.print(", Port: ");
                Serial.println(data.port);
                
                // Call user callback if registered
                if (onDistanceSensorCallback != nullptr) {
                    onDistanceSensorCallback(data.data.distanceSensor.distance, data.port);
                }
                break;
                
            case SENSOR_BUTTON:
                Serial.print("Main Task: Button callback - State: ");
                Serial.println((byte)data.data.button.state, HEX);
                
                // Call user callback if registered
                if (onButtonCallback != nullptr) {
                    onButtonCallback(data.data.button.state);
                }
                break;
                
            default:
                Serial.println("Main Task: Unknown sensor data type");
                break;
        }
    }
}

// ============================================================================
// CALLBACK REGISTRATION
// ============================================================================

void DuploHubExtended::setOnColorSensorCallback(ColorSensorCallback callback) {
    onColorSensorCallback = callback;
}

void DuploHubExtended::setOnDistanceSensorCallback(DistanceSensorCallback callback) {
    onDistanceSensorCallback = callback;
}

void DuploHubExtended::setOnButtonCallback(ButtonCallback callback) {
    onButtonCallback = callback;
}

// ============================================================================
// MAIN UPDATE LOOP (Extended with Sensor Data Processing)
// ============================================================================

void DuploHubExtended::update() {
    // Process connection state changes and fire callbacks
    if (xSemaphoreTake(connectionMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bool currentlyConnected = connectionState;
        
        // Handle connection state changes
        if (currentlyConnected && !wasConnected) {
            // Just connected
            if (onConnectedCallback != nullptr) {
                onConnectedCallback();
            }
        } else if (!currentlyConnected && wasConnected) {
            // Just disconnected
            if (onDisconnectedCallback != nullptr) {
                onDisconnectedCallback();
            }
        }
        
        wasConnected = currentlyConnected;
        xSemaphoreGive(connectionMutex);
    }
    
    // NEW: Process incoming sensor data from BLE task
    processSensorData();
    
    // Ensure BLE task is running
    ensureBLETaskRunning();
}
