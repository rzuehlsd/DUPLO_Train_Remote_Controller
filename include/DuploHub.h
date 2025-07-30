
/*
 * DuploHub.h
 *
 * Description:
 *   Thread-safe, event-driven abstraction for LEGO DUPLO Train Hub on ESP32.
 *   Provides BLE management, motor, LED, sound, and sensor operations with FreeRTOS support.
 *   Includes callback/event system, automatic connection management, and queue-based command processing.
 *
 * Author: Ralf Zühlsdorff
 * Copyright (c) 2025 Ralf Zühlsdorff
 * License: MIT License
 *
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

#ifndef DUPLO_HUB_H
#define DUPLO_HUB_H

#include "myLegoHub.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

namespace DuploEnums
{
    // Device types for Duplo Train Hub
    enum struct DuploTrainDeviceType
    {
        DUPLO_TRAIN_BASE_MOTOR = 41,
        DUPLO_TRAIN_BASE_SPEAKER = 42,
        DUPLO_TRAIN_BASE_COLOR_SENSOR = 43,
        DUPLO_TRAIN_BASE_SPEEDOMETER = 44,
        VOLTAGE_SENSOR = 20,
        LIGHT = 8
    };

    // Port enumeration for Duplo Train Hub
    // These ports correspond to the physical ports on the DUPLO Train Hub
    // and are used for device activation and communication.
    enum struct DuploTrainHubPort
    {
        MOTOR = 0x00,
        LED = 0x11,
        SPEAKER = 0x01,
        COLOR = 0x12,
        SPEEDOMETER = 0x13,
        VOLTAGE = 0x14
    };

    // Command types for thread-safe communication
    enum CommandType
    {
        CMD_ACTIVATE_RGB_LIGHT,
        CMD_ACTIVATE_BASE_SPEAKER,
        CMD_MOTOR_SPEED,
        CMD_STOP_MOTOR,
        CMD_SET_LED_COLOR,
        CMD_SET_HUB_NAME,
        CMD_PLAY_SOUND,
        CMD_ACTIVATE_COLOR_SENSOR,
        CMD_ACTIVATE_SPEED_SENSOR,
        CMD_ACTIVATE_VOLTAGE_SENSOR
    };

    // Enum for available Duplo sounds
    enum DuploSound
    {
        BRAKE = 3,
        STATION_DEPARTURE = 5,
        WATER_REFILL = 7,
        HORN = 9,
        STEAM = 10
    };

    // Enum for available Duplo colors
    enum DuploColor
    {
        BLACK = 0,
        PINK = 1,
        PURPLE = 2,
        BLUE = 3,
        LIGHT_BLUE = 4,
        CYAN = 5,
        GREEN = 6,
        YELLOW = 7,
        ORANGE = 8,
        RED = 9,
        WHITE = 10
    };

    // Enum for response types
    enum ResponseType
    {
        Detected_Color,
        Detected_Speed,
        Detected_Voltage
    };
}

// Callback function types
typedef void (*ConnectionCallback)();

// Add a typedef for the detected color callback
typedef void (*DetectedColorCallback)(DuploEnums::DuploColor);

// Add a typedef for the speed callback
typedef void (*DetectedSpeedCallback)(int detectedSpeed);

// Add a typedef for the voltage callback
typedef void (*DetectedVoltageCallback)(float detectedVoltage);

// Command structure for queue communication
typedef struct
{
    DuploEnums::CommandType type;
    union
    {
        struct
        {
            int speed;
        } motor;
        struct
        {
            DuploEnums::DuploColor color;
        } led;
        struct
        {
            char name[32]; // Fixed size for thread safety
        } hubName;
        struct
        {
            int soundId;
        } sound;
    } data;
} HubCommand;

// Response structure for queue communication
typedef struct
{
    DuploEnums::ResponseType type;
    union
    {
        struct
        {
            DuploEnums::DuploColor detectedColor;
        } colorResponse;
        struct
        {
            int detectedSpeed;
        } speedResponse;
        struct
        {
            float detectedVoltage;
        } voltageResponse;
        // Add other response types here as needed
    } data;
} HubResponse;

// Threshold definition for callback execution
#define VOLTAGE_THRESHOLD 0.1f
#define SPEED_THRESHOLD 5

// DuploHub class definition
// This class encapsulates the functionality of the LEGO DUPLO Train Hub
// It provides methods for connecting to the hub, controlling motors, lights, and sensors,
// and handling BLE communication in a thread-safe manner.
// The class uses FreeRTOS for task management and synchronization.


class DuploHub {
private:
    myLegoHub hub; ///< Underlying hardware interface
    byte motorPort;
    bool wasConnected;
    static DuploHub *instance;
    volatile bool connectionState;
    volatile bool connectingState;
    SemaphoreHandle_t connectionMutex;
    QueueHandle_t commandQueue;
    QueueHandle_t responseQueue;
    TaskHandle_t bleTaskHandle;
    // Callback functions
    ConnectionCallback onConnectedCallback;
    ConnectionCallback onDisconnectedCallback;
    DetectedColorCallback detectedColorCallback;
    DetectedSpeedCallback detectedSpeedCallback;
    DetectedVoltageCallback detectedVoltageCallback;
    // Internal task management
    void initFreeRTOS();
    void cleanupFreeRTOS();
    void updateConnectionState(bool connected, bool connecting);
    void processCommandQueue();
    void clearQueues();
    static void bleTaskWrapper(void *parameter);
    void bleTaskFunction();
protected:
    // Sensor callback registration for lpf2hub
    typedef void (*ColorSensorCallback)(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
    static void staticColorSensorCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
    typedef void (*SpeedSensorCallback)(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
    static void staticSpeedSensorCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
    typedef void (*VoltageSensorCallback)(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
    static void staticVoltageSensorCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
public:
    /**
     * @brief Default constructor.
     */
    DuploHub();

    /**
     * @brief Constructor with motor port selection.
     * @param port Motor port.
     */
    DuploHub(byte port);

    /**
     * @brief Destructor (cleans up FreeRTOS objects).
     */
    ~DuploHub();

    /**
     * @brief Initialize the hub and BLE connection.
     */
    void init();

    /**
     * @brief Initialize the hub with a specific BLE address.
     * @param address BLE address as string.
     */
    void init(const std::string &address);

    /**
     * @brief Check if hub is connected (thread-safe).
     * @return true if connected.
     */
    bool isConnected();

    /**
     * @brief Check if hub is connecting (thread-safe).
     * @return true if connecting.
     */
    bool isConnecting();

    /**
     * @brief Check if hub is disconnected (thread-safe).
     * @return true if disconnected.
     */
    bool isDisconnected();

    /**
     * @brief Start the BLE task on Core 0.
     */
    void startBLETask();

    /**
     * @brief Stop the BLE task.
     */
    void stopBLETask();

    /**
     * @brief Update BLE state (called from BLE task).
     */
    void updateBLE();

    /**
     * @brief Check if BLE task is running.
     * @return true if running.
     */
    bool isBLETaskRunning();

    /**
     * @brief Ensure BLE task is running (auto-recovery).
     */
    void ensureBLETaskRunning();

    /**
     * @brief Set the hub name.
     * @param name New hub name.
     */
    void setHubName(const char *name);

    /**
     * @brief Get the BLE address of the hub.
     * @return BLE address as string.
     */
    std::string getHubAddress();

    /**
     * @brief Get the hub name.
     * @return Hub name as string.
     */
    std::string getHubName();

    /**
     * @brief List all device ports.
     */
    void listDevicePorts();

    /**
     * @brief Set the motor port.
     * @param port Motor port.
     */
    void setMotorPort(byte port);

    /**
     * @brief Get the configured motor port.
     * @return Motor port.
     */
    byte getMotorPort();

    /**
     * @brief Set the motor speed.
     * @param speed Motor speed (-100..100).
     */
    void setMotorSpeed(int speed);

    /**
     * @brief Stop the motor immediately.
     */
    void stopMotor();

    /**
     * @brief Activate the base speaker port device.
     */
    void activateBaseSpeaker();

    /**
     * @brief Play a sound by sound ID.
     * @param soundId The sound ID to play.
     */
    void playSound(int soundId);

    /**
     * @brief Activate the RGB LED port device.
     */
    void activateRgbLight();

    /**
     * @brief Set the LED color.
     * @param color The color to set (DuploColor).
     */
    void setLedColor(DuploEnums::DuploColor color);

    /**
     * @brief Play a sound directly (low-level).
     * @param sound Sound ID.
     */
    void playSoundDirect(byte sound);

    /**
     * @brief Set LED color directly (low-level).
     * @param color Color value.
     */
    void setLedColorDirect(Color color);

    /**
     * @brief Activate base speaker directly (low-level).
     */
    void activateBaseSpeakerDirect();

    /**
     * @brief Activate RGB light directly (low-level).
     */
    void activateRgbLightDirect();

    /**
     * @brief Activate color sensor (legacy method).
     */
    void activateColorSensor();

    /**
     * @brief Activate speed sensor (legacy method).
     */
    void activateSpeedSensor();

    /**
     * @brief Activate voltage sensor.
     */
    void activateVoltageSensor();

    /**
     * @brief Register callback for hub connected event.
     * @param callback Function to call on connection.
     */
    void setOnConnectedCallback(ConnectionCallback callback);

    /**
     * @brief Register callback for hub disconnected event.
     * @param callback Function to call on disconnection.
     */
    void setOnDisconnectedCallback(ConnectionCallback callback);

    /**
     * @brief Register callback for detected color event.
     * @param callback Function to call on color detection.
     */
    void setDetectedColorCallback(DetectedColorCallback callback);

    /**
     * @brief Register callback for detected speed event.
     * @param callback Function to call on speed detection.
     */
    void setDetectedSpeedCallback(DetectedSpeedCallback callback);

    /**
     * @brief Register callback for detected voltage event.
     * @param callback Function to call on voltage detection.
     */
    void setDetectedVoltageCallback(DetectedVoltageCallback callback);

    /**
     * @brief Process response queue events in main task.
     */
    void processResponseQueue();

    /**
     * @brief BLE task event processing (call in main loop).
     */
    void update();
};

#endif // DUPLO_HUB_H
