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
#include "LegoinoCommon.h"

#undef DEBUG // Enable debug logging
#include "debug.h"

// Initialize static instance pointer
DuploHub *DuploHub::instance = nullptr;

// Default constructor
DuploHub::DuploHub() : motorPort((byte)DuploEnums::DuploTrainHubPort::MOTOR), wasConnected(false),
                       connectionState(false), connectingState(false),
                       connectionMutex(nullptr), commandQueue(nullptr), bleTaskHandle(nullptr),
                       onConnectedCallback(nullptr), onDisconnectedCallback(nullptr),
                       detectedColorCallback(nullptr), detectedSpeedCallback(nullptr),
                       detectedVoltageCallback(nullptr)
{
    instance = this; // Set static instance pointer
}

// Constructor with port specification
DuploHub::DuploHub(byte port) : motorPort(port), wasConnected(false),
                                connectionState(false), connectingState(false),
                                connectionMutex(nullptr), commandQueue(nullptr), bleTaskHandle(nullptr),
                                onConnectedCallback(nullptr), onDisconnectedCallback(nullptr),
                                detectedColorCallback(nullptr), detectedSpeedCallback(nullptr),
                                detectedVoltageCallback(nullptr)
{
    instance = this; // Set static instance pointer
}

// Destructor
DuploHub::~DuploHub()
{
    cleanupFreeRTOS();
    if (instance == this)
    {
        instance = nullptr; // Clear static instance pointer
    }
}

// Initialize FreeRTOS objects
void DuploHub::initFreeRTOS()
{
    // Create mutex for thread-safe access to connection state
    connectionMutex = xSemaphoreCreateMutex();
    if (connectionMutex == nullptr)
    {
        DEBUG_LOG("ERROR: Failed to create connection mutex");
    }

    // Create command queue for thread-safe communication
    commandQueue = xQueueCreate(10, sizeof(HubCommand)); // Queue size: 10 commands
    if (commandQueue == nullptr)
    {
        DEBUG_LOG("ERROR: Failed to create command queue");
    }

    responseQueue = xQueueCreate(100, sizeof(HubResponse)); // Queue size: 10 response entries
    if (responseQueue == nullptr)
    {
        DEBUG_LOG("ERROR: Failed to create response queue");
    }

    DEBUG_LOG("DuploHub: FreeRTOS objects initialized");
    Serial.flush();
}

// Cleanup FreeRTOS objects
void DuploHub::cleanupFreeRTOS()
{
    // Stop BLE task if running
    stopBLETask();

    // Delete mutex
    if (connectionMutex != nullptr)
    {
        vSemaphoreDelete(connectionMutex);
        connectionMutex = nullptr;
    }

    // Delete command queue
    if (commandQueue != nullptr)
    {
        vQueueDelete(commandQueue);
        commandQueue = nullptr;
        DEBUG_LOG("DuploHub: Command queue cleaned up");
    }

    // Cleanup response queue
    if (responseQueue != nullptr)
    {
        vQueueDelete(responseQueue);
        responseQueue = nullptr;
        DEBUG_LOG("DuploHub: Response queue cleaned up");
    }

    DEBUG_LOG("DuploHub: FreeRTOS objects cleaned up");
    Serial.flush();
}

// Update connection state (thread-safe)
void DuploHub::updateConnectionState(bool connected, bool connecting)
{
    if (connectionMutex != nullptr)
    {
        xSemaphoreTake(connectionMutex, portMAX_DELAY);
        connectionState = connected;
        connectingState = connecting;
        xSemaphoreGive(connectionMutex);
    }
}

// Initialize the hub (thread-safe - can be called from main loop)
void DuploHub::init()
{
    DEBUG_LOG("DuploHub: Initializing ...");
    initFreeRTOS();
    delay(500);
    startBLETask();
    delay(500);
    DEBUG_LOG("DuploHub: Initialization complete");
}

// Initialize the hub with specific address (thread-safe)
void DuploHub::init(const std::string &address)
{
    DEBUG_LOG("WARNING: init(address) should not be called directly - BLE task handles initialization");
    // For Phase 2, we'll store the address preference but let BLE task handle it
    // This could be enhanced in Phase 3 to set a preferred address
}

// Check if hub is connected (thread-safe)
bool DuploHub::isConnected()
{
    if (connectionMutex != nullptr)
    {
        xSemaphoreTake(connectionMutex, portMAX_DELAY);
        bool state = connectionState;
        xSemaphoreGive(connectionMutex);
        return state;
    }
    return hub.isConnected(); // Fallback if mutex not initialized
}

// Check if hub is connecting (thread-safe)
bool DuploHub::isConnecting()
{
    if (connectionMutex != nullptr)
    {
        xSemaphoreTake(connectionMutex, portMAX_DELAY);
        bool state = connectingState;
        xSemaphoreGive(connectionMutex);
        return state;
    }
    return hub.isConnecting(); // Fallback if mutex not initialized
}

// Check if hub is disconnected (thread-safe)
bool DuploHub::isDisconnected()
{
    return !isConnecting() && !isConnected();
}

// Set hub name (thread-safe)
void DuploHub::setHubName(const char *name)
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_SET_HUB_NAME;
        strncpy(cmd.data.hubName.name, name, sizeof(cmd.data.hubName.name) - 1);
        cmd.data.hubName.name[sizeof(cmd.data.hubName.name) - 1] = '\0'; // Ensure null termination

        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue hub name command");
        }
    }
    else
    {
        char hubName[strlen(name) + 1];
        strcpy(hubName, name);
        hub.setHubName(hubName);
    }
}

// Get hub address
std::string DuploHub::getHubAddress()
{
    return hub.getHubAddress().toString();
}

// Get hub name
std::string DuploHub::getHubName()
{
    return hub.getHubName();
}

// Set LED color (thread-safe)
void DuploHub::setLedColor(DuploEnums::DuploColor color)
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_SET_LED_COLOR;
        cmd.data.led.color = (DuploEnums::DuploColor)color;

        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue LED color command");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue is not initialized");
    }
}

// Set motor speed (thread-safe)
void DuploHub::setMotorSpeed(int speed)
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_MOTOR_SPEED;
        cmd.data.motor.speed = speed;

        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue motor speed command");
        }
    }
    else
    {
        hub.setBasicMotorSpeed(motorPort, speed);
        DEBUG_LOG("DuploHub: setBasicMotorSpeed executed at: %lu", millis());
    }
}

// Stop motor (thread-safe)
void DuploHub::stopMotor()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_STOP_MOTOR;

        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue stop motor command");
        }
    }
    else
    {
        hub.stopBasicMotor(motorPort);
    }
}

// Play a sound on the Duplo Hub (thread-safe)
void DuploHub::playSound(int soundId)
{
    DEBUG_LOG("DuploHub: playSound called with soundId: %d", soundId);
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_PLAY_SOUND;
        cmd.data.sound.soundId = soundId;

        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue play sound command");
        }
        else
        {
            DEBUG_LOG("DuploHub: Sound command queued successfully");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue not initialized - cannot play sound");
    }
}

// Activate RGB light (thread-safe)
void DuploHub::activateRgbLight()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_ACTIVATE_RGB_LIGHT;

        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue activate RGB light command");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue is not initialized");
    }
}

// Activate base speaker (thread-safe)
void DuploHub::activateBaseSpeaker()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_ACTIVATE_BASE_SPEAKER;

        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue activate base speaker command");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue is not initialized");
    }
}

void DuploHub::activateColorSensor()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_ACTIVATE_COLOR_SENSOR;

        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue activate color sensor command");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue is not initialized");
    }
}

void DuploHub::activateSpeedSensor()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_ACTIVATE_SPEED_SENSOR;

        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue activate speed sensor command");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue is not initialized");
    }
}

void DuploHub::activateVoltageSensor()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = DuploEnums::CMD_ACTIVATE_VOLTAGE_SENSOR;

        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue activate voltage sensor command");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue is not initialized");
    }
}

// Static callback function that can be passed to activatePortDevice
void DuploHub::staticColorSensorCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData)
{
    static int lastColor = -1;

    myLegoHub *myHub = (myLegoHub *)hub;
    if (deviceType == DeviceType::DUPLO_TRAIN_BASE_COLOR_SENSOR)
    {
        int detectedColor = myHub->parseColor(pData);
        if (detectedColor >= DuploEnums::DuploColor::BLACK && detectedColor <= DuploEnums::DuploColor::WHITE)
        {
            // Only process valid color values
            if (lastColor != detectedColor)
            {
                DEBUG_LOG("(%ld) Static Color Callback - Last Color: %d , Detected Color: %d", millis(), lastColor, detectedColor);
                lastColor = detectedColor;

                // Access the singleton instance to get the response queue
                if (instance != nullptr && instance->responseQueue != nullptr)
                {
                    HubResponse response;
                    response.type = DuploEnums::ResponseType::Detected_Color;
                    response.data.colorResponse.detectedColor = (DuploEnums::DuploColor)detectedColor;

                    if (xQueueSend(instance->responseQueue, &response, pdMS_TO_TICKS(100)) != pdTRUE)
                    {
                        DEBUG_LOG("WARNING: Failed to queue color sensor response from static callback");
                    }
                }
                else
                {
                    DEBUG_LOG("ERROR: Cannot access response queue from static color callback");
                }
            }
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Unsupported device type for color sensor callback");
    }
}

// Static callback function that can be passed to activatePortDevice for speed sensor
void DuploHub::staticSpeedSensorCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData)
{
    static int lastSpeed = -1;

    myLegoHub *myHub = (myLegoHub *)hub;
    if (deviceType == DeviceType::DUPLO_TRAIN_BASE_SPEEDOMETER)
    {
        int detectedSpeed = myHub->parseSpeedometer(pData);
        if (abs(lastSpeed - detectedSpeed) > SPEED_THRESHOLD)
        { // Only report significant speed changes
            DEBUG_LOG("(%ld) Static Speed Callback - Last Speed: %d , Detected Speed: %d", millis(), lastSpeed, detectedSpeed);
            lastSpeed = detectedSpeed;

            // Access the singleton instance to get the response queue
            if (instance != nullptr && instance->responseQueue != nullptr)
            {
                HubResponse response;
                response.type = DuploEnums::ResponseType::Detected_Speed;
                response.data.speedResponse.detectedSpeed = myHub->MapSpeedometer(detectedSpeed);

                if (xQueueSend(instance->responseQueue, &response, pdMS_TO_TICKS(100)) != pdTRUE)
                {
                    DEBUG_LOG("WARNING: Failed to queue speed sensor response from static callback");
                }
            }
            else
            {
                DEBUG_LOG("ERROR: Cannot access response queue from static speed callback");
            }
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Unsupported device type for speed sensor callback");
    }
}

// Static callback function that can be passed to activatePortDevice for voltage sensor
void DuploHub::staticVoltageSensorCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData)
{
    static float lastVoltage = -1.0f;

    myLegoHub *myHub = (myLegoHub *)hub;
    if (deviceType == DeviceType::VOLTAGE_SENSOR)
    {
        float detectedVoltage = (float)myHub->parseVoltageSensor(pData);
        if (abs(lastVoltage - detectedVoltage) > VOLTAGE_THRESHOLD)
        { // Only report significant voltage changes
            DEBUG_LOG("(%ld) Static Voltage Callback - Last Voltage: %.2f , Detected Voltage: %.2f", millis(), lastVoltage, detectedVoltage);
            lastVoltage = detectedVoltage;

            // Access the singleton instance to get the response queue
            if (instance != nullptr && instance->responseQueue != nullptr)
            {
                HubResponse response;
                response.type = DuploEnums::ResponseType::Detected_Voltage;
                response.data.voltageResponse.detectedVoltage = detectedVoltage;

                if (xQueueSend(instance->responseQueue, &response, pdMS_TO_TICKS(100)) != pdTRUE)
                {
                    DEBUG_LOG("WARNING: Failed to queue voltage sensor response from static callback");
                }
            }
            else
            {
                DEBUG_LOG("ERROR: Cannot access response queue from static voltage callback");
            }
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Unsupported device type for voltage sensor callback");
    }
}

// Set motor port
void DuploHub::setMotorPort(byte port)
{
    motorPort = port;
}

// Get motor port
byte DuploHub::getMotorPort()
{
    return motorPort;
}

void DuploHub::listDevicePorts()
{
    DEBUG_LOG("Listing Duplo Train device ports:");
    DEBUG_LOG("Motor Port: %d", (int)hub.getPortForDeviceType((byte)DuploEnums::DuploTrainDeviceType::DUPLO_TRAIN_BASE_MOTOR));
    DEBUG_LOG("Speaker Port: %d", (int)hub.getPortForDeviceType((byte)DuploEnums::DuploTrainDeviceType::DUPLO_TRAIN_BASE_SPEAKER));
    DEBUG_LOG("Color Sensor Port: %d", (int)hub.getPortForDeviceType((byte)DuploEnums::DuploTrainDeviceType::DUPLO_TRAIN_BASE_COLOR_SENSOR));
    DEBUG_LOG("Speed Sensor Port: %d", (int)hub.getPortForDeviceType((byte)DuploEnums::DuploTrainDeviceType::DUPLO_TRAIN_BASE_SPEEDOMETER));
    DEBUG_LOG("Voltage Port: %d", (int)hub.getPortForDeviceType((byte)DuploEnums::DuploTrainDeviceType::VOLTAGE_SENSOR));
}

// Set callback for when hub connects
void DuploHub::setOnConnectedCallback(ConnectionCallback callback)
{
    onConnectedCallback = callback;
    DEBUG_LOG("DuploHub: OnConnected callback set");
}

// Set callback for when hub disconnects
void DuploHub::setOnDisconnectedCallback(ConnectionCallback callback)
{
    onDisconnectedCallback = callback;
    DEBUG_LOG("DuploHub: OnDisconnected callback set");
}

// Implement the function to register the detected color callback
void DuploHub::setDetectedColorCallback(DetectedColorCallback callback)
{
    detectedColorCallback = callback;
    DEBUG_LOG("DuploHub: DetectedColor callback set");
}

// Implement the function to register the detected speed callback
void DuploHub::setDetectedSpeedCallback(DetectedSpeedCallback callback)
{
    detectedSpeedCallback = callback;
    DEBUG_LOG("DuploHub: DetectedSpeed callback set");
}

// Implement the function to register the detected voltage callback
void DuploHub::setDetectedVoltageCallback(DetectedVoltageCallback callback)
{
    detectedVoltageCallback = callback;
    DEBUG_LOG("DuploHub: DetectedVoltage callback set");
}

// Start BLE task
void DuploHub::startBLETask()
{
    if (bleTaskHandle == nullptr)
    {
        BaseType_t result = xTaskCreatePinnedToCore(
            bleTaskWrapper, // Task function
            "BLE_Task",     // Name
            4096,           // Stack size (4KB)
            this,           // Parameter (this instance)
            2,              // Priority (higher than main loop)
            &bleTaskHandle, // Task handle
            0               // CPU core 0
        );

        if (result == pdPASS)
        {
            DEBUG_LOG("DuploHub: BLE task started successfully");
        }
        else
        {
            DEBUG_LOG("ERROR: Failed to start BLE task");
            bleTaskHandle = nullptr;
        }
    }
    else
    {
        DEBUG_LOG("WARNING: BLE task is already running");
    }
}

// Stop BLE task
void DuploHub::stopBLETask()
{
    if (bleTaskHandle != nullptr)
    {
        vTaskDelete(bleTaskHandle);
        bleTaskHandle = nullptr;
        DEBUG_LOG("DuploHub: BLE task stopped");
    }
}

// Check if BLE task is running
bool DuploHub::isBLETaskRunning()
{
    return bleTaskHandle != nullptr;
}

// Ensure BLE task is running (auto-recovery)
void DuploHub::ensureBLETaskRunning()
{
    if (!isBLETaskRunning())
    {
        DEBUG_LOG("DuploHub: BLE task not running, attempting to restart...");
        delay(200);
        startBLETask();
    }
}

// Static wrapper for BLE task (required for FreeRTOS)
void DuploHub::bleTaskWrapper(void *parameter)
{
    DuploHub *instance = static_cast<DuploHub *>(parameter);
    instance->bleTaskFunction();
}

// BLE task function
void DuploHub::bleTaskFunction()
{
    DEBUG_LOG("BLE Task: Started successfully");
    Serial.flush();

    unsigned long lastConnectionCheck = 0;
    const unsigned long CONNECTION_CHECK_INTERVAL = 1000; // Check connection every 1 second

    while (true)
    {
        unsigned long currentTime = millis();

        // Handle BLE operations at regular intervals
        if (currentTime - lastConnectionCheck >= CONNECTION_CHECK_INTERVAL)
        {
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
void DuploHub::updateBLE()
{
    static bool wasEverConnected = false; // Track if we ever had a connection (BLE task context)
    static bool lastConnected = false;    // Track last connection state for callback

    // Get current BLE states from Lpf2Hub
    bool hubConnected = hub.isConnected();
    bool hubConnecting = hub.isConnecting();
    bool hubDisconnected = !hubConnecting && !hubConnected;

    // Update thread-safe state variables
    updateConnectionState(hubConnected, hubConnecting);

    // BLE Connection Management
    if (hubDisconnected)
    {
        if (!wasEverConnected)
        {
            DEBUG_LOG("BLE Task: Attempting initial connection to hub...");
        }
        else
        {
            DEBUG_LOG("BLE Task: Attempting to reconnect to hub...");
        }
        hub.init();
    }

    // Handle connection process
    if (hubConnecting)
    {
        hub.connectHub();
        if (hub.isConnected())
        {
            wasEverConnected = true;
            DEBUG_LOG("BLE Task: Connected to HUB");
            Serial.flush();
            DEBUG_LOG("BLE Task: Hub address: %s", hub.getHubAddress().toString().c_str());
            DEBUG_LOG("BLE Task: Hub name: %s", hub.getHubName().c_str());
        }
        else
        {
            DEBUG_LOG("BLE Task: Failed to connect to HUB");
            Serial.flush();
        }
    }

    // Connection state change detection and callback invocation (moved from update())
    if (hubConnected && !lastConnected)
    {
        DEBUG_LOG("BLE Task: Connection state changed: CONNECTED");
        clearQueues();
        if (onConnectedCallback != nullptr)
        {
            DEBUG_LOG("DuploHub: Calling onConnectedCallback from BLE task...");
            onConnectedCallback();
            DEBUG_LOG("DuploHub: onConnectedCallback finished");
        }
        else
        {
            DEBUG_LOG("DuploHub: onConnectedCallback is nullptr");
        }
    }
    else if (!hubConnected && lastConnected)
    {
        clearQueues();
        DEBUG_LOG("BLE Task: Connection state changed: DISCONNECTED");
        if (onDisconnectedCallback != nullptr)
        {
            DEBUG_LOG("DuploHub: Calling onDisconnectedCallback from BLE task...");
            onDisconnectedCallback();
            DEBUG_LOG("DuploHub: onDisconnectedCallback finished");
        }
        else
        {
            DEBUG_LOG("DuploHub: onDisconnectedCallback is nullptr");
        }
    }
    lastConnected = hubConnected;
}

// Process commands from the queue (runs in BLE task)
void DuploHub::processCommandQueue()
{
    if (commandQueue == nullptr)
        return;

    HubCommand cmd;

    // Process all available commands (non-blocking)
    while (xQueueReceive(commandQueue, &cmd, 0) == pdTRUE)
    {
        // Only process commands if connected
        if (!hub.isConnected())
        {
            DEBUG_LOG("BLE Task: Skipping command - hub not connected");
            continue;
        }

        switch (cmd.type)
        {
        case DuploEnums::CMD_MOTOR_SPEED:
            DEBUG_LOG("BLE Task: Setting motor speed to %d", cmd.data.motor.speed);
            hub.setBasicMotorSpeed((byte)DuploEnums::DuploTrainHubPort::MOTOR, cmd.data.motor.speed);
            delay(20); // Reduced delay for faster motor response
            DEBUG_LOG("DuploHub: setBasicMotorSpeed completed at: %lu", millis());
            break;

        case DuploEnums::CMD_STOP_MOTOR:
            DEBUG_LOG("BLE Task: Stopping motor");
            hub.stopBasicMotor((byte)DuploEnums::DuploTrainHubPort::MOTOR);
            delay(20); // Reduced delay for faster stop
            DEBUG_LOG("DuploHub: stopBasicMotor completed at: %lu", millis());
            break;

        case DuploEnums::CMD_SET_LED_COLOR:
            DEBUG_LOG("BLE Task: Setting LED color to %d", cmd.data.led.color);
            hub.setLedColor((Color)cmd.data.led.color);
            delay(200); // Ensure LED color command is processed
            DEBUG_LOG("DuploHub: setLEDColor completed at: %lu", millis());
            break;

        case DuploEnums::CMD_SET_HUB_NAME:
            DEBUG_LOG("BLE Task: Setting hub name to %s", cmd.data.hubName.name);
            hub.setHubName(cmd.data.hubName.name);
            delay(200); // Ensure hub name command is processed
            DEBUG_LOG("DuploHub: setHubName completed at: %lu", millis());
            break;

        case DuploEnums::CMD_PLAY_SOUND:
            DEBUG_LOG("BLE Task: Playing sound with ID %d", cmd.data.sound.soundId);
            hub.playSound((byte)cmd.data.sound.soundId);
            delay(200); // Ensure sound command is processed
            DEBUG_LOG("DuploHub: playSound completed at: %lu", millis());
            break;

        case DuploEnums::CMD_ACTIVATE_RGB_LIGHT:
            DEBUG_LOG("BLE Task: Activating RGB light");
            hub.activateRgbLight();
            delay(200); // Ensure RGB light command is processed
            DEBUG_LOG("DuploHub: activateRgbLight completed");
            break;

        case DuploEnums::CMD_ACTIVATE_BASE_SPEAKER:
            DEBUG_LOG("BLE Task: Activating base speaker");
            hub.activateBaseSpeaker();
            delay(200); // Ensure base speaker command is processed
            DEBUG_LOG("DuploHub: activateBaseSpeaker completed");
            break;

        case DuploEnums::CMD_ACTIVATE_COLOR_SENSOR:
            DEBUG_LOG("BLE Task: Activating color sensor");
            hub.activatePortDevice((byte)DuploEnums::DuploTrainHubPort::COLOR, staticColorSensorCallback);
            delay(200);
            DEBUG_LOG("DuploHub: setColorSensorCallback completed");
            break;

        case DuploEnums::CMD_ACTIVATE_SPEED_SENSOR:
            DEBUG_LOG("BLE Task: Activating speed sensor");
            hub.activatePortDevice((byte)DuploEnums::DuploTrainHubPort::SPEEDOMETER, staticSpeedSensorCallback);
            delay(200);
            DEBUG_LOG("DuploHub: setSpeedSensorCallback completed");
            break;
        case DuploEnums::CMD_ACTIVATE_VOLTAGE_SENSOR:
            DEBUG_LOG("BLE Task: Activating voltage sensor");
            hub.activatePortDevice((byte)DuploEnums::DuploTrainHubPort::VOLTAGE, staticVoltageSensorCallback);
            delay(200);
            DEBUG_LOG("DuploHub: setVoltageSensorCallback completed");
            break;

        default:
            DEBUG_LOG("BLE Task: Unknown command type");
            break;
        }
    }
}

// Ensure alignment with the header file by adding support for HubResponse
void DuploHub::processResponseQueue()
{
    if (responseQueue == nullptr)
        return;

    HubResponse response;

    // Process all available responses (non-blocking)
    while (xQueueReceive(responseQueue, &response, 0) == pdTRUE)
    {
        switch (response.type)
        {
        case DuploEnums::ResponseType::Detected_Color:
            DEBUG_LOG("Detected Color: %d", response.data.colorResponse.detectedColor);
            if (detectedColorCallback != nullptr)
            {
                detectedColorCallback(response.data.colorResponse.detectedColor);
                DEBUG_LOG("Detected Color Callback executed");
            }
            break;

        case DuploEnums::ResponseType::Detected_Speed:
            DEBUG_LOG("Detected Speed: %d", response.data.speedResponse.detectedSpeed);
            if (detectedSpeedCallback != nullptr)
            {
                detectedSpeedCallback(response.data.speedResponse.detectedSpeed);
                DEBUG_LOG("Detected Speed Callback executed");
            }
            break;

        case DuploEnums::ResponseType::Detected_Voltage:
            DEBUG_LOG("Detected Voltage: %.2f V", response.data.voltageResponse.detectedVoltage);
            if (detectedVoltageCallback != nullptr)
            {
                detectedVoltageCallback(response.data.voltageResponse.detectedVoltage);
                DEBUG_LOG("Detected Voltage Callback executed");
            }
            break;

        default:
            DEBUG_LOG("Unknown response type");
            break;
        }
    }
}

// Main update method - handles callbacks only (non-blocking)
void DuploHub::update()
{
    // Ensure BLE task is running (auto-recovery)
    static unsigned long lastTaskCheck = 0;
    if (millis() - lastTaskCheck > 5000)
    { // Check every 5 seconds
        ensureBLETaskRunning();
        lastTaskCheck = millis();
    }
    // No connection state change or callback logic here anymore; handled in BLE task
    wasConnected = isConnected();
}

// Clear all pending commands and responses in the queues
void DuploHub::clearQueues()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        while (xQueueReceive(commandQueue, &cmd, 0) == pdTRUE)
        {
            // Discard all commands
        }
    }
    if (responseQueue != nullptr)
    {
        HubResponse response;
        while (xQueueReceive(responseQueue, &response, 0) == pdTRUE)
        {
            // Discard all responses
        }
    }
}
