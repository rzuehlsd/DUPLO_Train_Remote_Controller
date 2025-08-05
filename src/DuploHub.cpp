
/**
 * @file DuploHub.cpp
 * @brief Implementation of the DuploHub class for LEGO DUPLO Train Hub control on ESP32.
 *
 * This file provides the implementation for the DuploHub class, which encapsulates all
 * functionality required to control a LEGO DUPLO Train Hub using the ESP32 platform.
 *
 * Features:
 * - Thread-safe command queuing for motor, LED, and sound control
 * - Optimized BLE task responsiveness and reduced latency
 * - Automatic connection management and recovery
 * - Dual-core FreeRTOS architecture for robust, responsive operation
 * - Modular callback system for sensors and events
 *
 * @author Michael
 * @date 2025
 *
 * @copyright
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "Arduino.h"
#include "DuploHub.h"
#include "LegoinoCommon.h"
#include "esp_timer.h"

#undef DEBUG// Enable debug logging
#include "debug.h"

// Initialize static instance pointer
DuploHub *DuploHub::instance = nullptr;

/**
 * @brief Default constructor for DuploHub.
 *
 * Initializes the DuploHub instance with the default motor port and resets all state variables.
 */
DuploHub::DuploHub() : motorPort((byte)DuploEnums::DuploTrainHubPort::MOTOR), wasConnected(false),
                       connectionState(false), connectingState(false),
                       connectionMutex(nullptr), commandQueue(nullptr), bleTaskHandle(nullptr),
                       onConnectedCallback(nullptr), onDisconnectedCallback(nullptr),
                       detectedColorCallback(nullptr), detectedSpeedCallback(nullptr),
                       detectedVoltageCallback(nullptr)
{
    instance = this; // Set static instance pointer
}

/**
 * @brief Constructor for DuploHub with a specific motor port.
 * @param port The port number to use for the motor.
 */
DuploHub::DuploHub(byte port) : motorPort(port), wasConnected(false),
                                connectionState(false), connectingState(false),
                                connectionMutex(nullptr), commandQueue(nullptr), bleTaskHandle(nullptr),
                                onConnectedCallback(nullptr), onDisconnectedCallback(nullptr),
                                detectedColorCallback(nullptr), detectedSpeedCallback(nullptr),
                                detectedVoltageCallback(nullptr)
{
    instance = this; // Set static instance pointer
}

/**
 * @brief Destructor for DuploHub.
 *
 * Cleans up FreeRTOS resources and resets the static instance pointer.
 */
DuploHub::~DuploHub()
{
    cleanupFreeRTOS();
    if (instance == this)
    {
        instance = nullptr; // Clear static instance pointer
    }
}

/**
 * @brief Initialize FreeRTOS objects (mutexes and queues) for thread-safe operation.
 *
 * Creates mutexes and queues for command and response handling.
 */
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

/**
 * @brief Clean up FreeRTOS objects (mutexes and queues).
 *
 * Deletes all created mutexes and queues and stops the BLE task if running.
 */
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

/**
 * @brief Update the connection state variables in a thread-safe manner.
 * @param connected True if the hub is connected.
 * @param connecting True if the hub is in the process of connecting.
 */
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

/**
 * @brief Initialize the DuploHub (thread-safe, can be called from main loop).
 *
 * Sets up FreeRTOS resources and starts the BLE task.
 */
void DuploHub::init()
{
    DEBUG_LOG("DuploHub: Initializing ...");
    initFreeRTOS();
    delay(500);
    startBLETask();
    delay(500);
    DEBUG_LOG("DuploHub: Initialization complete");
}

/**
 * @brief Initialize the DuploHub with a specific BLE address (not recommended).
 * @param address The BLE address to use for the hub.
 *
 * Note: BLE task handles initialization; this is for future use.
 */
void DuploHub::init(const std::string &address)
{
    DEBUG_LOG("WARNING: init(address) should not be called directly - BLE task handles initialization");
    // For Phase 2, we'll store the address preference but let BLE task handle it
    // This could be enhanced in Phase 3 to set a preferred address
}

/**
 * @brief Check if the hub is connected (thread-safe).
 * @return True if connected, false otherwise.
 */
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

/**
 * @brief Check if the hub is currently connecting (thread-safe).
 * @return True if connecting, false otherwise.
 */
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

/**
 * @brief Check if the hub is disconnected (thread-safe).
 * @return True if disconnected, false otherwise.
 */
bool DuploHub::isDisconnected()
{
    return !isConnecting() && !isConnected();
}


/**
 * @brief Get the BLE address of the hub.
 * @return The hub's BLE address as a string.
 */
std::string DuploHub::getHubAddress()
{
    return hub.getHubAddress().toString();
}

/**
 * @brief Get the name of the hub.
 * @return The hub's name as a string.
 */
std::string DuploHub::getHubName()
{
    return hub.getHubName();
}

/**
 * @brief Enable or disable command recording
 * @param enable True to start recording commands, false to stop
 */
void DuploHub::recordCommands(bool enable)
{
    recordCommandsEnabled = enable;
    if (enable)
    {
        commandBuffer.clear(); // Clear existing commands when starting new recording
        DEBUG_LOG("DuploHub: Command recording ENABLED. Buffer cleared.");
    }
    else
    {
        DEBUG_LOG("DuploHub: Command recording DISABLED. Buffer contains %zu commands.", commandBuffer.size());
    }
}

/**
 * @brief Start replaying all previously recorded commands with normalized timestamps
 * The first command timestamp is set to 0 and all other timestamps are adjusted
 * to maintain relative timing differences
 */
void DuploHub::replayCommands()
{
    if (commandBuffer.empty())
    {
        DEBUG_LOG("DuploHub: No commands to replay. Buffer is empty.");
        return;
    }
    
    if (replaying)
    {
        DEBUG_LOG("DuploHub: Replay already in progress. Stopping current replay first.");
        stopReplay();
    }
    
    recordCommandsEnabled = false;
    
    // Normalize timestamps - set first command to time 0 and adjust all others relative to it
    if (!commandBuffer.empty())
    {
        originalStartTime = commandBuffer[0].timestamp;
        
        for (auto& cmd : commandBuffer)
        {
            cmd.timestamp -= originalStartTime;  // Normalize to start at 0
        }
        
        DEBUG_LOG("DuploHub: Normalized %zu commands. First command now at timestamp 0.", commandBuffer.size());
    }
    
    // Initialize replay state
    replaying = true;
    replayIndex = 0;
    replayStartTime = esp_timer_get_time();
    
    DEBUG_LOG("DuploHub: Started replay of %zu recorded commands. Use getNextReplayCommand() to retrieve commands.", commandBuffer.size());
}

/**
 * @brief Get the next command to replay if it's time to execute it
 * @return Pointer to the next command if ready, nullptr if no command ready or replay finished
 */
const HubCommand* DuploHub::getNextReplayCommand()
{
    if (!replaying || replayIndex >= commandBuffer.size())
    {
        return nullptr;
    }
    
    uint64_t currentTime = esp_timer_get_time();
    uint64_t elapsedTime = currentTime - replayStartTime;
    
    const HubCommand& nextCmd = commandBuffer[replayIndex];
    
    // Check if it's time to play this command
    if (elapsedTime >= nextCmd.timestamp)
    {
        replayIndex++;
        
        DEBUG_LOG("DuploHub: Retrieved replay command %zu/%zu (type: %d) at elapsed time %llu us", 
                  replayIndex, commandBuffer.size(), nextCmd.type, elapsedTime);
        
        // Check if replay is complete
        if (replayIndex >= commandBuffer.size())
        {
            DEBUG_LOG("DuploHub: Replay completed. All %zu commands processed.", commandBuffer.size());
            replaying = false;
        }
        
        return &nextCmd;
    }
    
    return nullptr; // Not yet time to play this command
}

/**
 * @brief Check if command replay is currently active
 * @return true if replay is in progress, false otherwise
 */
bool DuploHub::isReplayActive() const
{
    return replaying;
}

/**
 * @brief Stop the current command replay
 */
void DuploHub::stopReplay()
{
    if (replaying)
    {
        DEBUG_LOG("DuploHub: Stopping replay at command %zu/%zu", replayIndex, commandBuffer.size());
        replaying = false;
        replayIndex = 0;
        replayStartTime = 0;
    }
    else
    {
        DEBUG_LOG("DuploHub: No active replay to stop.");
    }
}

/**
 * @brief Wrapper around xQueueSend that optionally records commands to commandBuffer
 * @param cmd The command to send and optionally record
 * @param timeout Timeout for queue send operation
 * @return Result of xQueueSend operation
 */
BaseType_t DuploHub::sendCommand(const HubCommand& cmd, TickType_t timeout)
{
    // Store command in buffer if recording is enabled
    if (recordCommandsEnabled)
    {
        commandBuffer.push_back(cmd);
        DEBUG_LOG("DuploHub: Command %d recorded to buffer. Buffer size: %zu", cmd.type, commandBuffer.size());
    }
    
    // Send command to queue
    return xQueueSend(commandQueue, &cmd, timeout);
}

/**
 * @brief Set the hub's name (thread-safe).
 * @param name The new name for the hub.
 */
void DuploHub::setHubName(const char *name)
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = CommandType::CMD_SET_HUB_NAME;
        cmd.timestamp = esp_timer_get_time(); // Add timestamp in microseconds
        strncpy(cmd.data.hubName.name, name, sizeof(cmd.data.hubName.name) - 1);
        cmd.data.hubName.name[sizeof(cmd.data.hubName.name) - 1] = '\0'; // Ensure null termination

        if (sendCommand(cmd, pdMS_TO_TICKS(100)) != pdTRUE)
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


/**
 * @brief Set the LED color on the hub (thread-safe).
 * @param color The color to set (DuploEnums::DuploColor).
 */
void DuploHub::setLedColor(DuploEnums::DuploColor color)
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = CommandType::CMD_SET_LED_COLOR;
        cmd.timestamp = esp_timer_get_time(); // Add timestamp in microseconds
        cmd.data.led.color = (DuploEnums::DuploColor)color;

        if (sendCommand(cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue LED color command");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue is not initialized");
    }
}

/**
 * @brief Set the motor speed (thread-safe).
 * @param speed The speed to set for the motor.
 */
void DuploHub::setMotorSpeed(int speed)
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = CommandType::CMD_MOTOR_SPEED;
        cmd.timestamp = esp_timer_get_time(); // Add timestamp in microseconds
        cmd.data.motor.speed = speed;

        if (sendCommand(cmd, pdMS_TO_TICKS(100)) != pdTRUE)
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

/**
 * @brief Stop the motor (thread-safe).
 */
void DuploHub::stopMotor()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = CommandType::CMD_STOP_MOTOR;
        cmd.timestamp = esp_timer_get_time(); // Add timestamp in microseconds

        if (sendCommand(cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue stop motor command");
        }
    }
    else
    {
        hub.stopBasicMotor(motorPort);
    }
}

/**
 * @brief Play a sound on the Duplo Hub (thread-safe).
 * @param soundId The ID of the sound to play.
 */
void DuploHub::playSound(int soundId)
{
    DEBUG_LOG("DuploHub: playSound called with soundId: %d", soundId);
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = CommandType::CMD_PLAY_SOUND;
        cmd.timestamp = esp_timer_get_time(); // Add timestamp in microseconds
        cmd.data.sound.soundId = soundId;

        if (sendCommand(cmd, pdMS_TO_TICKS(100)) != pdTRUE)
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

/**
 * @brief Activate the RGB light on the hub (thread-safe).
 */
void DuploHub::activateRgbLight()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = CommandType::CMD_ACTIVATE_RGB_LIGHT;
        cmd.timestamp = esp_timer_get_time(); // Add timestamp in microseconds

        if (sendCommand(cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue activate RGB light command");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue is not initialized");
    }
}

/**
 * @brief Activate the base speaker on the hub (thread-safe).
 */
void DuploHub::activateBaseSpeaker()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = CommandType::CMD_ACTIVATE_BASE_SPEAKER;
        cmd.timestamp = esp_timer_get_time(); // Add timestamp in microseconds

        if (sendCommand(cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue activate base speaker command");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue is not initialized");
    }
}

/**
 * @brief Activate the color sensor on the hub (thread-safe).
 */
void DuploHub::activateColorSensor()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = CommandType::CMD_ACTIVATE_COLOR_SENSOR;
        cmd.timestamp = esp_timer_get_time(); // Add timestamp in microseconds

        if (sendCommand(cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue activate color sensor command");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue is not initialized");
    }
}

/**
 * @brief Activate the speed sensor on the hub (thread-safe).
 */
void DuploHub::activateSpeedSensor()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = CommandType::CMD_ACTIVATE_SPEED_SENSOR;
        cmd.timestamp = esp_timer_get_time(); // Add timestamp in microseconds

        if (sendCommand(cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue activate speed sensor command");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue is not initialized");
    }
}

/**
 * @brief Activate the voltage sensor on the hub (thread-safe).
 */
void DuploHub::activateVoltageSensor()
{
    if (commandQueue != nullptr)
    {
        HubCommand cmd;
        cmd.type = CommandType::CMD_ACTIVATE_VOLTAGE_SENSOR;
        cmd.timestamp = esp_timer_get_time(); // Add timestamp in microseconds

        if (sendCommand(cmd, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            DEBUG_LOG("WARNING: Failed to queue activate voltage sensor command");
        }
    }
    else
    {
        DEBUG_LOG("ERROR: Command queue is not initialized");
    }
}

/**
 * @brief Static callback for color sensor events (used by activatePortDevice).
 * @param hub Pointer to the hub instance.
 * @param portNumber The port number of the sensor.
 * @param deviceType The type of device triggering the callback.
 * @param pData Pointer to the sensor data.
 */
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
                    response.type = ResponseType::Detected_Color;
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

/**
 * @brief Static callback for speed sensor events (used by activatePortDevice).
 * @param hub Pointer to the hub instance.
 * @param portNumber The port number of the sensor.
 * @param deviceType The type of device triggering the callback.
 * @param pData Pointer to the sensor data.
 */
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
                response.type = ResponseType::Detected_Speed;
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

/**
 * @brief Static callback for voltage sensor events (used by activatePortDevice).
 * @param hub Pointer to the hub instance.
 * @param portNumber The port number of the sensor.
 * @param deviceType The type of device triggering the callback.
 * @param pData Pointer to the sensor data.
 */
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
                response.type = ResponseType::Detected_Voltage;
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

/**
 * @brief Set the motor port for the hub.
 * @param port The port number to assign to the motor.
 */
void DuploHub::setMotorPort(byte port)
{
    motorPort = port;
}

/**
 * @brief Get the current motor port number.
 * @return The port number assigned to the motor.
 */

byte DuploHub::getMotorPort()
{
    return motorPort;
}


/**
 * @brief List all device ports for the Duplo Train hub.
 */
void DuploHub::listDevicePorts()
{
    DEBUG_LOG("Listing Duplo Train device ports:");
    DEBUG_LOG("Motor Port: %d", (int)hub.getPortForDeviceType((byte)DuploEnums::DuploTrainDeviceType::DUPLO_TRAIN_BASE_MOTOR));
    DEBUG_LOG("Speaker Port: %d", (int)hub.getPortForDeviceType((byte)DuploEnums::DuploTrainDeviceType::DUPLO_TRAIN_BASE_SPEAKER));
    DEBUG_LOG("Color Sensor Port: %d", (int)hub.getPortForDeviceType((byte)DuploEnums::DuploTrainDeviceType::DUPLO_TRAIN_BASE_COLOR_SENSOR));
    DEBUG_LOG("Speed Sensor Port: %d", (int)hub.getPortForDeviceType((byte)DuploEnums::DuploTrainDeviceType::DUPLO_TRAIN_BASE_SPEEDOMETER));
    DEBUG_LOG("Voltage Port: %d", (int)hub.getPortForDeviceType((byte)DuploEnums::DuploTrainDeviceType::VOLTAGE_SENSOR));
}

/**
 * @brief Set the callback function to be called when the hub connects.
 * @param callback The function to call on connection.
 */
void DuploHub::setOnConnectedCallback(ConnectionCallback callback)
{
    onConnectedCallback = callback;
    DEBUG_LOG("DuploHub: OnConnected callback set");
}

/**
 * @brief Set the callback function to be called when the hub disconnects.
 * @param callback The function to call on disconnection.
 */
void DuploHub::setOnDisconnectedCallback(ConnectionCallback callback)
{
    onDisconnectedCallback = callback;
    DEBUG_LOG("DuploHub: OnDisconnected callback set");
}

/**
 * @brief Set the callback function for detected color events.
 * @param callback The function to call when a color is detected.
 */
void DuploHub::setDetectedColorCallback(DetectedColorCallback callback)
{
    detectedColorCallback = callback;
    DEBUG_LOG("DuploHub: DetectedColor callback set");
}

/**
 * @brief Set the callback function for detected speed events.
 * @param callback The function to call when a speed is detected.
 */
void DuploHub::setDetectedSpeedCallback(DetectedSpeedCallback callback)
{
    detectedSpeedCallback = callback;
    DEBUG_LOG("DuploHub: DetectedSpeed callback set");
}

/**
 * @brief Set the callback function for detected voltage events.
 * @param callback The function to call when a voltage is detected.
 */
void DuploHub::setDetectedVoltageCallback(DetectedVoltageCallback callback)
{
    detectedVoltageCallback = callback;
    DEBUG_LOG("DuploHub: DetectedVoltage callback set");
}

/**
 * @brief Start the BLE task on core 0 (if not already running).
 */
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

/**
 * @brief Stop the BLE task if it is running.
 */
void DuploHub::stopBLETask()
{
    if (bleTaskHandle != nullptr)
    {
        vTaskDelete(bleTaskHandle);
        bleTaskHandle = nullptr;
        DEBUG_LOG("DuploHub: BLE task stopped");
    }
}

/**
 * @brief Check if the BLE task is currently running.
 * @return True if running, false otherwise.
 */
bool DuploHub::isBLETaskRunning()
{
    return bleTaskHandle != nullptr;
}

/**
 * @brief Ensure the BLE task is running; restart if not (auto-recovery).
 */
void DuploHub::ensureBLETaskRunning()
{
    if (!isBLETaskRunning())
    {
        DEBUG_LOG("DuploHub: BLE task not running, attempting to restart...");
        delay(200);
        startBLETask();
    }
}

/**
 * @brief Static wrapper for the BLE task (required for FreeRTOS).
 * @param parameter Pointer to the DuploHub instance.
 */
void DuploHub::bleTaskWrapper(void *parameter)
{
    DuploHub *instance = static_cast<DuploHub *>(parameter);
    instance->bleTaskFunction();
}

/**
 * @brief BLE task function that handles BLE operations and command processing.
 */
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

/**
 * @brief Perform BLE operations, connection management, and callback invocation (runs in BLE task).
 */
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

/**
 * @brief Process all commands from the command queue (runs in BLE task).
 */
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
        case CommandType::CMD_MOTOR_SPEED:
            DEBUG_LOG("BLE Task: Setting motor speed to %d", cmd.data.motor.speed);
            hub.setBasicMotorSpeed((byte)DuploEnums::DuploTrainHubPort::MOTOR, cmd.data.motor.speed);
            delay(20); // Reduced delay for faster motor response
            DEBUG_LOG("DuploHub: setBasicMotorSpeed completed at: %lu", millis());
            break;

        case CommandType::CMD_STOP_MOTOR:
            DEBUG_LOG("BLE Task: Stopping motor");
            hub.stopBasicMotor((byte)DuploEnums::DuploTrainHubPort::MOTOR);
            delay(20); // Reduced delay for faster stop
            DEBUG_LOG("DuploHub: stopBasicMotor completed at: %lu", millis());
            break;

        case CommandType::CMD_SET_LED_COLOR:
            DEBUG_LOG("BLE Task: Setting LED color to %d", cmd.data.led.color);
            hub.setLedColor((Color)cmd.data.led.color);
            delay(200); // Ensure LED color command is processed
            DEBUG_LOG("DuploHub: setLEDColor completed at: %lu", millis());
            break;

        case CommandType::CMD_SET_HUB_NAME:
            DEBUG_LOG("BLE Task: Setting hub name to %s", cmd.data.hubName.name);
            hub.setHubName(cmd.data.hubName.name);
            delay(200); // Ensure hub name command is processed
            DEBUG_LOG("DuploHub: setHubName completed at: %lu", millis());
            break;

        case CommandType::CMD_PLAY_SOUND:
            DEBUG_LOG("BLE Task: Playing sound with ID %d", cmd.data.sound.soundId);
            hub.playSound((byte)cmd.data.sound.soundId);
            delay(200); // Ensure sound command is processed
            DEBUG_LOG("DuploHub: playSound completed at: %lu", millis());
            break;

        case CommandType::CMD_ACTIVATE_RGB_LIGHT:
            DEBUG_LOG("BLE Task: Activating RGB light");
            hub.activateRgbLight();
            delay(200); // Ensure RGB light command is processed
            DEBUG_LOG("DuploHub: activateRgbLight completed");
            break;

        case CommandType::CMD_ACTIVATE_BASE_SPEAKER:
            DEBUG_LOG("BLE Task: Activating base speaker");
            hub.activateBaseSpeaker();
            delay(200); // Ensure base speaker command is processed
            DEBUG_LOG("DuploHub: activateBaseSpeaker completed");
            break;

        case CommandType::CMD_ACTIVATE_COLOR_SENSOR:
            DEBUG_LOG("BLE Task: Activating color sensor");
            hub.activatePortDevice((byte)DuploEnums::DuploTrainHubPort::COLOR, staticColorSensorCallback);
            delay(200);
            DEBUG_LOG("DuploHub: setColorSensorCallback completed");
            break;

        case CommandType::CMD_ACTIVATE_SPEED_SENSOR:
            DEBUG_LOG("BLE Task: Activating speed sensor");
            hub.activatePortDevice((byte)DuploEnums::DuploTrainHubPort::SPEEDOMETER, staticSpeedSensorCallback);
            delay(200);
            DEBUG_LOG("DuploHub: setSpeedSensorCallback completed");
            break;
        case CommandType::CMD_ACTIVATE_VOLTAGE_SENSOR:
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

/**
 * @brief Process all responses from the response queue and invoke registered callbacks.
 */
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
        case Detected_Color:
            DEBUG_LOG("Detected Color: %d", response.data.colorResponse.detectedColor);
            if (detectedColorCallback != nullptr)
            {
                detectedColorCallback(response.data.colorResponse.detectedColor);
                DEBUG_LOG("Detected Color Callback executed");
            }
            break;

        case Detected_Speed:
            DEBUG_LOG("Detected Speed: %d", response.data.speedResponse.detectedSpeed);
            if (detectedSpeedCallback != nullptr)
            {
                detectedSpeedCallback(response.data.speedResponse.detectedSpeed);
                DEBUG_LOG("Detected Speed Callback executed");
            }
            break;

        case Detected_Voltage:
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

/**
 * @brief Main update method; ensures BLE task is running and updates connection state.
 */
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

/**
 * @brief Clear all pending commands and responses in the command and response queues.
 */
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
