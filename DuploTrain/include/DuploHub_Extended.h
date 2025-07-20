/**
 * DuploHub_Extended - Extended version with bidirectional sensor data support
 * 
 * This shows how to extend the current architecture to support sensor data
 * flowing from BLE task back to the main application via FreeRTOS queues
 * 
 * Architecture:
 * - Commands: Main Task (Core 1) → BLE Task (Core 0) via commandQueue
 * - Sensor Data: BLE Task (Core 0) → Main Task (Core 1) via sensorQueue
 * 
 * (c) Copyright 2025
 * Released under MIT License
 * 
 */

#ifndef DUPLO_HUB_EXTENDED_H
#define DUPLO_HUB_EXTENDED_H

#include "Lpf2Hub.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// Callback function types
typedef void (*ConnectionCallback)();
typedef void (*ColorSensorCallback)(int color, byte port);
typedef void (*DistanceSensorCallback)(int distance, byte port);
typedef void (*ButtonCallback)(ButtonState state);

// Command types for thread-safe communication (Main → BLE)
enum CommandType {
    CMD_MOTOR_SPEED,
    CMD_STOP_MOTOR,
    CMD_SET_LED_COLOR,
    CMD_SET_HUB_NAME,
    CMD_ACTIVATE_COLOR_SENSOR,
    CMD_ACTIVATE_DISTANCE_SENSOR,
    CMD_ACTIVATE_BUTTON
};

// Sensor data types (BLE → Main)
enum SensorDataType {
    SENSOR_COLOR,
    SENSOR_DISTANCE,
    SENSOR_BUTTON,
    SENSOR_CONNECTION_STATE
};

// Command structure for Main → BLE communication
typedef struct {
    CommandType type;
    union {
        struct {
            int speed;
        } motor;
        struct {
            Color color;
        } led;
        struct {
            char name[32];  // Fixed size for thread safety
        } hubName;
        struct {
            byte port;
        } sensor;
    } data;
} HubCommand;

// Sensor data structure for BLE → Main communication
typedef struct {
    SensorDataType type;
    byte port;
    unsigned long timestamp;  // When the data was received
    union {
        struct {
            int color;
        } colorSensor;
        struct {
            int distance;
        } distanceSensor;
        struct {
            ButtonState state;
        } button;
        struct {
            bool connected;
            bool connecting;
        } connection;
    } data;
} SensorData;

class DuploHubExtended {
private:
    Lpf2Hub hub;
    byte motorPort;
    bool wasConnected;  // Track previous connection state
    
    // Thread-safe state variables
    volatile bool connectionState;
    volatile bool connectingState;
    
    // FreeRTOS synchronization objects
    SemaphoreHandle_t connectionMutex;
    QueueHandle_t commandQueue;    // Main → BLE (commands)
    QueueHandle_t sensorQueue;     // BLE → Main (sensor data)
    TaskHandle_t bleTaskHandle;
    
    // Callback functions
    ConnectionCallback onConnectedCallback;
    ConnectionCallback onDisconnectedCallback;
    ColorSensorCallback onColorSensorCallback;
    DistanceSensorCallback onDistanceSensorCallback;
    ButtonCallback onButtonCallback;
    
    // Private methods for task management
    void initFreeRTOS();
    void cleanupFreeRTOS();
    void updateConnectionState(bool connected, bool connecting);
    void processCommandQueue();
    void processSensorData();
    static void bleTaskWrapper(void* parameter);
    void bleTaskFunction();
    
    // Static callback wrappers for Lpf2Hub integration
    static void colorSensorCallbackWrapper(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
    static void distanceSensorCallbackWrapper(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
    static void buttonCallbackWrapper(void *hub, HubPropertyReference hubProperty, uint8_t *pData);

protected:
    // Thread-safe implementation methods (used internally)
    void setHubName_ThreadSafe(const char* name);
    void setLedColor_ThreadSafe(Color color);
    void setMotorSpeed_ThreadSafe(int speed);
    void stopMotor_ThreadSafe();
    void activateColorSensor_ThreadSafe(byte port);
    void activateDistanceSensor_ThreadSafe(byte port);
    void activateButton_ThreadSafe();
    
    // Internal sensor data queuing (called from BLE task)
    void queueSensorData(const SensorData& data);
    
public:
    // Constructor & Destructor
    DuploHubExtended();
    DuploHubExtended(byte port);
    ~DuploHubExtended();  // Destructor for cleanup
    
    // Initialization and connection management
    void init();
    void init(const std::string& address);
    bool connect();
    bool isConnected();           // Thread-safe version
    bool isConnecting();          // Thread-safe version
    bool isDisconnected();        // Thread-safe version
    
    // Task management
    void startBLETask();
    void stopBLETask();
    void updateBLE();             // Called from BLE task
    bool isBLETaskRunning();
    void ensureBLETaskRunning();  // Auto-recovery mechanism
    
    // Hub information and settings
    std::string getHubAddress();
    std::string getHubName();
    
    // Motor control
    void setMotorPort(byte port);
    byte getMotorPort();
    
    // Sensor activation (thread-safe)
    void activateColorSensor(byte port);
    void activateDistanceSensor(byte port);
    void activateButton();
    
    // Legacy methods (for backward compatibility)
    void setHubName(const char* name) { setHubName_ThreadSafe(name); }
    void setLedColor(Color color) { setLedColor_ThreadSafe(color); }
    void setMotorSpeed(int speed) { setMotorSpeed_ThreadSafe(speed); }
    void stopMotor() { stopMotor_ThreadSafe(); }
    
    // Callback registration
    void setOnConnectedCallback(ConnectionCallback callback);
    void setOnDisconnectedCallback(ConnectionCallback callback);
    void setOnColorSensorCallback(ColorSensorCallback callback);
    void setOnDistanceSensorCallback(DistanceSensorCallback callback);
    void setOnButtonCallback(ButtonCallback callback);
    
    // Main update loop - now processes both connection callbacks AND sensor data
    void update();
};

#endif // DUPLO_HUB_EXTENDED_H
