/**
 * @file DuploHub.h
 * @brief High-level controller for the LEGO DUPLO Train Hub running on ESP32.
 *
 * Declares the `DuploHub` class, which wraps Legoino primitives in a thread-safe, event-driven
 * interface tailored for the dual-core ESP32 environment. Responsibilities include BLE lifecycle
 * management, command queuing, sensor event processing, and record/replay support.
 *
 * @author Ralf Zühlsdorff
 * @date 2025
 *
 * @copyright
 * MIT License
 */

#ifndef DUPLO_HUB_H
#define DUPLO_HUB_H


#include <vector>
#include "myLegoHub.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

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

// Command structure for queue communication
typedef struct
{
    CommandType type;
    long timestamp; // Timestamp for command execution order
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

// Enum for response types
enum ResponseType
{
    Detected_Color,
    Detected_Speed,
    Detected_Voltage
};

// Response structure for queue communication
typedef struct
{
    ResponseType type;
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

class DuploHub
{
private:
    myLegoHub hub; ///< Underlying hardware interface
    byte motorPort;
    std::string hubMACAddress;
    bool wasConnected;
    static DuploHub *instance;
    volatile bool connectionState;
    volatile bool connectingState;
    SemaphoreHandle_t connectionMutex;
    QueueHandle_t commandQueue;
    QueueHandle_t responseQueue;
    TaskHandle_t bleTaskHandle;
    
    // Command buffer for record / replay processing
    std::vector<HubCommand> commandBuffer;
    
    // Replay state management
    bool replaying = false;
    size_t replayIndex = 0;
    int64_t replayStartTime = 0;
    int64_t originalStartTime = 0;
    int64_t sequenceDuration = 0;  // Total duration of command sequence
    int replayCycle = 0;           // Current replay cycle number
    bool recordCommandsEnabled = false;

    // Callback functions
    ConnectionCallback onConnectedCallback;
    ConnectionCallback onDisconnectedCallback;
    DetectedColorCallback detectedColorCallback;
    DetectedSpeedCallback detectedSpeedCallback;
    DetectedVoltageCallback detectedVoltageCallback;

    // Internal task management
    /**
     * @brief Create FreeRTOS primitives required for hub operation.
     */
    void initFreeRTOS();

    /**
     * @brief Tear down FreeRTOS objects created by the hub.
     */
    void cleanupFreeRTOS();

    /**
     * @brief Update cached connection flags under mutex protection.
     *
     * @param connected Current connection state reported by Legoino.
     * @param connecting True while an outbound connection attempt is in progress.
     */
    void updateConnectionState(bool connected, bool connecting);

    /**
     * @brief Pull commands from the queue and forward them to the hub.
     */
    void processCommandQueue();

    /**
     * @brief Flush pending commands and responses.
     */
    void clearQueues();
    
    // Command queue wrapper for recording functionality
    /**
     * @brief Enqueue a command, optionally recording it for replay.
     *
     * @param cmd Command to be sent to the hub task.
     * @param timeout Maximum time the caller may block waiting for space in the queue.
     * @return `pdTRUE` on success, or `errQUEUE_FULL` if the queue is saturated.
     */
    BaseType_t sendCommand(const HubCommand& cmd, TickType_t timeout);

    /**
     * @brief Lazily create the timer that debounces color sensor readings.
     *
     * @return True when the timer exists and is ready to start.
     */
    static bool ensureColorTimerInitialized();
    
    /**
     * @brief FreeRTOS entry point for the BLE management task.
     */
    static void bleTaskWrapper(void *parameter);

    /**
     * @brief Worker routine executed by the BLE management task.
     */
    void bleTaskFunction();

    /**
     * @brief Spawn the BLE task if it is not currently running.
     */
    void startBLETask();

    /**
     * @brief Request the BLE task to stop and wait for cleanup.
     */
    void stopBLETask();

    /**
     * @brief Advance connection state machine internal to the BLE task.
     */
    void updateBLE();

    /**
     * @brief Ensure the BLE task is alive before queuing work.
     */
    void ensureBLETaskRunning();

protected:
    // Sensor callback registration for lpf2hub
    typedef void (*ColorSensorCallback)(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);

    /**
     * @brief FreeRTOS-safe entry point for color sensor notifications.
     */
    static void staticColorSensorCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);

    /**
     * @brief Timer hook that emits a color event once readings stabilize.
     */
    static void colorStabilityTimerCallback(void *arg);

    typedef void (*SpeedSensorCallback)(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);

    /**
     * @brief FreeRTOS-safe entry point for speed sensor notifications.
     */
    static void staticSpeedSensorCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);

    typedef void (*VoltageSensorCallback)(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);

    /**
     * @brief FreeRTOS-safe entry point for voltage sensor notifications.
     */
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
     * @brief Check if BLE task is running (thread-safe).
     * @return true if BLE task is running.
     */
    bool isBLETaskRunning();


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
     * @brief Enable or disable command recording for replay functionality
     * @param enable True to start recording commands, false to stop recording
     */
    void recordCommands(bool enable);

    /**
     * @brief Start replaying all previously recorded commands with normalized timestamps
     * The first command timestamp is set to 0 and all other timestamps are adjusted
     * to maintain relative timing differences
     */
    void replayCommands();
    
    /**
     * @brief Get the next command to replay if it's time to execute it
     * @return Pointer to the next command if ready, nullptr if no command ready or replay finished
     */
    const HubCommand* getNextReplayCommand();
    
    /**
     * @brief Check if replay is currently active
     * @return true if replaying, false otherwise
     */
    bool isReplayActive() const;
    
    /**
     * @brief Stop the current replay operation
     */
    void stopReplay();

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
