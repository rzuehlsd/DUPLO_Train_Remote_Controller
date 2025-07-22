/**
 * DuploHub - A class to encapsulate LEGO DUPLO Train Hub functionality
 * 
 * Features:
 * - Thread-safe command queuing for motor, LED, and sound control
 * - Optimized BLE task responsiveness and reduced latency
 * - Automatic connection management and recovery
 * 
 * (c) Copyright 2025
 * Released under MIT License
 */

#include "Arduino.h"
#include "DuploHub.h"

// Default constructor
DuploHub::DuploHub() : motorPort((byte)PoweredUpHubPort::A), wasConnected(false),
                       connectionState(false), connectingState(false),
                       connectionMutex(nullptr), commandQueue(nullptr), bleTaskHandle(nullptr),
                       onConnectedCallback(nullptr), onDisconnectedCallback(nullptr) {
    initFreeRTOS();
}

// Constructor with port specification
DuploHub::DuploHub(byte port) : motorPort(port), wasConnected(false),
                                connectionState(false), connectingState(false),
                                connectionMutex(nullptr), commandQueue(nullptr), bleTaskHandle(nullptr),
                                onConnectedCallback(nullptr), onDisconnectedCallback(nullptr) {
    initFreeRTOS();
}

// Destructor
DuploHub::~DuploHub() {
    cleanupFreeRTOS();
}

// Initialize FreeRTOS objects
void DuploHub::initFreeRTOS() {
    // Create mutex for thread-safe access to connection state
    connectionMutex = xSemaphoreCreateMutex();
    if (connectionMutex == nullptr) {
        Serial.println("ERROR: Failed to create connection mutex");
    }
    
    // Create command queue for thread-safe communication
    commandQueue = xQueueCreate(10, sizeof(HubCommand));  // Queue size: 10 commands
    if (commandQueue == nullptr) {
        Serial.println("ERROR: Failed to create command queue");
    }
    
    Serial.println("DuploHub: FreeRTOS objects initialized");
}

// Cleanup FreeRTOS objects
void DuploHub::cleanupFreeRTOS() {
    // Stop BLE task if running
    stopBLETask();
    
    // Delete mutex
    if (connectionMutex != nullptr) {
        vSemaphoreDelete(connectionMutex);
        connectionMutex = nullptr;
    }
    
    // Delete queue
    if (commandQueue != nullptr) {
        vQueueDelete(commandQueue);
        commandQueue = nullptr;
    }
    
    Serial.println("DuploHub: FreeRTOS objects cleaned up");
}

// Update connection state (thread-safe)
void DuploHub::updateConnectionState(bool connected, bool connecting) {
    if (connectionMutex != nullptr) {
        xSemaphoreTake(connectionMutex, portMAX_DELAY);
        connectionState = connected;
        connectingState = connecting;
        xSemaphoreGive(connectionMutex);
    }
}

// Initialize the hub (thread-safe - can be called from main loop)
void DuploHub::init() {
    Serial.println("WARNING: init() should not be called directly - BLE task handles initialization");
    // The actual initialization is now handled by the BLE task
}

// Initialize the hub with specific address (thread-safe)
void DuploHub::init(const std::string& address) {
    Serial.println("WARNING: init(address) should not be called directly - BLE task handles initialization");
    // For Phase 2, we'll store the address preference but let BLE task handle it
    // This could be enhanced in Phase 3 to set a preferred address
}

// Legacy connect method (kept for compatibility but deprecated)
// Note: Actual connection is now handled by BLE task
bool DuploHub::connect() {
    Serial.println("WARNING: connect() is deprecated - connection handled by BLE task");
    return isConnected();
}

// Check if hub is connected (thread-safe)
bool DuploHub::isConnected() {
    if (connectionMutex != nullptr) {
        xSemaphoreTake(connectionMutex, portMAX_DELAY);
        bool state = connectionState;
        xSemaphoreGive(connectionMutex);
        return state;
    }
    return hub.isConnected(); // Fallback if mutex not initialized
}

// Check if hub is connecting (thread-safe)
bool DuploHub::isConnecting() {
    if (connectionMutex != nullptr) {
        xSemaphoreTake(connectionMutex, portMAX_DELAY);
        bool state = connectingState;
        xSemaphoreGive(connectionMutex);
        return state;
    }
    return hub.isConnecting(); // Fallback if mutex not initialized
}

// Check if hub is disconnected (thread-safe)
bool DuploHub::isDisconnected() {
    return !isConnecting() && !isConnected();
}

// Set hub name (thread-safe)
void DuploHub::setHubName_ThreadSafe(const char* name) {
    if (commandQueue != nullptr) {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_SET_HUB_NAME;
        strncpy(cmd.data.hubName.name, name, sizeof(cmd.data.hubName.name) - 1);
        cmd.data.hubName.name[sizeof(cmd.data.hubName.name) - 1] = '\0'; // Ensure null termination
        
        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            Serial.println("WARNING: Failed to queue hub name command");
        }
    } else {
        // Fallback to direct call if queue not initialized
        char hubName[strlen(name) + 1];
        strcpy(hubName, name);
        hub.setHubName(hubName);
    }
}

// Get hub address
std::string DuploHub::getHubAddress() {
    return hub.getHubAddress().toString();
}

// Get hub name
std::string DuploHub::getHubName() {
    return hub.getHubName();
}

// Set LED color (thread-safe)
void DuploHub::setLedColor_ThreadSafe(DuploEnums::DuploColor color) {
    if (commandQueue != nullptr) {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_SET_LED_COLOR;
        cmd.data.led.color = (DuploEnums::DuploColor) color;
        
        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            Serial.println("WARNING: Failed to queue LED color command");
        }
    } else {
        Serial.println("ERROR: Command queue is not initialized");
    }
}

// Set motor speed (thread-safe)
void DuploHub::setMotorSpeed_ThreadSafe(int speed) {
    if (commandQueue != nullptr) {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_MOTOR_SPEED;
        cmd.data.motor.speed = speed;
        
        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            Serial.println("WARNING: Failed to queue motor speed command");
        }
    } else {
        // Fallback to direct call if queue not initialized
        hub.setBasicMotorSpeed(motorPort, speed);
        Serial.print("DuploHub: setBasicMotorSpeed executed at: ");
        Serial.println(millis());
    }
}

// Stop motor (thread-safe)
void DuploHub::stopMotor_ThreadSafe() {
    if (commandQueue != nullptr) {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_STOP_MOTOR;
        
        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            Serial.println("WARNING: Failed to queue stop motor command");
        }
    } else {
        // Fallback to direct call if queue not initialized
        hub.stopBasicMotor(motorPort);
    }
}


// Play a sound on the Duplo Hub (thread-safe)
void DuploHub::playSound_ThreadSafe(int soundId) {
    if (commandQueue != nullptr) {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_PLAY_SOUND;
        cmd.data.sound.soundId = soundId;

        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            Serial.println("WARNING: Failed to queue play sound command");
        }
    } else {
        Serial.println("ERROR: Command queue not initialized");
    }
}

// Set motor port
void DuploHub::setMotorPort(byte port) {
    motorPort = port;
}

// Get motor port
byte DuploHub::getMotorPort() {
    return motorPort;
}

// Set callback for when hub connects
void DuploHub::setOnConnectedCallback(ConnectionCallback callback) {
    onConnectedCallback = callback;
}

// Set callback for when hub disconnects
void DuploHub::setOnDisconnectedCallback(ConnectionCallback callback) {
    onDisconnectedCallback = callback;
}

// Start BLE task
void DuploHub::startBLETask() {
    if (bleTaskHandle == nullptr) {
        BaseType_t result = xTaskCreatePinnedToCore(
            bleTaskWrapper,        // Task function
            "BLE_Task",           // Name
            4096,                 // Stack size (4KB)
            this,                 // Parameter (this instance)
            2,                    // Priority (higher than main loop)
            &bleTaskHandle,       // Task handle
            0                     // CPU core 0
        );
        
        if (result == pdPASS) {
            Serial.println("DuploHub: BLE task started successfully");
        } else {
            Serial.println("ERROR: Failed to start BLE task");
            bleTaskHandle = nullptr;
        }
    } else {
        Serial.println("WARNING: BLE task is already running");
    }
}

// Stop BLE task
void DuploHub::stopBLETask() {
    if (bleTaskHandle != nullptr) {
        vTaskDelete(bleTaskHandle);
        bleTaskHandle = nullptr;
        Serial.println("DuploHub: BLE task stopped");
    }
}

// Check if BLE task is running
bool DuploHub::isBLETaskRunning() {
    return bleTaskHandle != nullptr;
}

// Ensure BLE task is running (auto-recovery)
void DuploHub::ensureBLETaskRunning() {
    if (!isBLETaskRunning()) {
        Serial.println("DuploHub: BLE task not running, attempting to restart...");
        startBLETask();
    }
}

// Static wrapper for BLE task (required for FreeRTOS)
void DuploHub::bleTaskWrapper(void* parameter) {
    DuploHub* instance = static_cast<DuploHub*>(parameter);
    instance->bleTaskFunction();
}

// BLE task function
void DuploHub::bleTaskFunction() {
    Serial.println("BLE Task: Started successfully");
    
    unsigned long lastConnectionCheck = 0;
    const unsigned long CONNECTION_CHECK_INTERVAL = 1000; // Check connection every 1 second
    
    while (true) {
        unsigned long currentTime = millis();
        
        // Handle BLE operations at regular intervals
        if (currentTime - lastConnectionCheck >= CONNECTION_CHECK_INTERVAL) {
            updateBLE();
            lastConnectionCheck = currentTime;
        }
        
        // Process command queue more frequently for responsiveness
        processCommandQueue();
        
        // Small delay to prevent task from hogging CPU
        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms delay for good responsiveness
    }
}

// BLE operations (runs in BLE task)
void DuploHub::updateBLE() {
    static bool wasEverConnected = false;  // Track if we ever had a connection (BLE task context)
    
    // Get current BLE states from Lpf2Hub
    bool hubConnected = hub.isConnected();
    bool hubConnecting = hub.isConnecting();
    bool hubDisconnected = !hubConnecting && !hubConnected;
    
    // Update thread-safe state variables
    updateConnectionState(hubConnected, hubConnecting);
    
    // BLE Connection Management
    if (hubDisconnected) {
        if (!wasEverConnected) {
            Serial.println("BLE Task: Attempting initial connection to hub...");
        } else {
            Serial.println("BLE Task: Attempting to reconnect to hub...");
        }
        hub.init();
    }
    
    // Handle connection process
    if (hubConnecting) {
        hub.connectHub();
        if (hub.isConnected()) {
            wasEverConnected = true;
            Serial.println("BLE Task: Connected to HUB");
            Serial.print("BLE Task: Hub address: ");
            Serial.println(hub.getHubAddress().toString().c_str());
            Serial.print("BLE Task: Hub name: ");
            Serial.println(hub.getHubName().c_str());
        } else {
            Serial.println("BLE Task: Failed to connect to HUB");
        }
    }
    
    // Process command queue
    processCommandQueue();
}

// Process commands from the queue (runs in BLE task)
void DuploHub::processCommandQueue() {
    if (commandQueue == nullptr) return;
    
    HubCommand cmd;
    
    // Process all available commands (non-blocking)
    while (xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
        // Only process commands if connected
        if (!hub.isConnected()) {
            Serial.println("BLE Task: Skipping command - hub not connected");
            continue;
        }
        
        switch (cmd.type) {
            case DuploEnums::CMD_MOTOR_SPEED:
                Serial.print("BLE Task: Setting motor speed to ");
                Serial.println(cmd.data.motor.speed);
                Serial.flush();
                hub.setBasicMotorSpeed(motorPort, cmd.data.motor.speed);
                Serial.print("DuploHub: setBasicMotorSpeed completed at: ");
                Serial.println(millis());
                Serial.flush();
                break;
                
            case DuploEnums::CMD_STOP_MOTOR:
                Serial.println("BLE Task: Stopping motor");
                hub.stopBasicMotor(motorPort);
                break;
                
            case DuploEnums::CMD_SET_LED_COLOR:
                Serial.print("BLE Task: Setting LED color to ");
                Serial.println(cmd.data.led.color);
                Serial.flush();
                hub.setLedColor((Color)cmd.data.led.color);
                Serial.print("DuploHub: setLEDColor completed at: ");
                Serial.println(millis());
                Serial.flush();
                break;
                
            case DuploEnums::CMD_SET_HUB_NAME:
                Serial.print("BLE Task: Setting hub name to ");
                Serial.println(cmd.data.hubName.name);
                hub.setHubName(cmd.data.hubName.name);
                break;
                
            case DuploEnums::CMD_PLAY_SOUND:
                Serial.print("BLE Task: Playing sound with ID ");
                Serial.println(cmd.data.sound.soundId);
                Serial.flush();
                hub.playSound(cmd.data.sound.soundId);
                Serial.print("DuploHub: playSound completed at: ");
                Serial.println(millis());
                Serial.flush();
                break;
                
            default:
                Serial.println("BLE Task: Unknown command type");
                break;
        }
    }
}

// Main update method - handles callbacks only (non-blocking)
void DuploHub::update() {
    // Ensure BLE task is running (auto-recovery)
    static unsigned long lastTaskCheck = 0;
    if (millis() - lastTaskCheck > 5000) { // Check every 5 seconds
        ensureBLETaskRunning();
        lastTaskCheck = millis();
    }
    
    // Check for connection state changes and trigger callbacks
    bool currentlyConnected = isConnected();
    
    // Handle connection state changes
    if (currentlyConnected && !wasConnected) {
        // Connection established
        Serial.println("Train hub is connected");
        if (onConnectedCallback != nullptr) {
            onConnectedCallback();
        }
    } else if (!currentlyConnected && wasConnected) {
        // Connection lost
        Serial.println("Train hub is disconnected");
        if (onDisconnectedCallback != nullptr) {
            onDisconnectedCallback();
        }
    }
    
    // Update the connection state for next iteration
    wasConnected = currentlyConnected;
}
