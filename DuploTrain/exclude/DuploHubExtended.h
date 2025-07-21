#ifndef DUPLO_HUBEXTENDED_H
#define DUPLO_HUBEXTENDED_H
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

#include "DuploHub.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// Additional callback function types specific to DuploHubExtended
typedef void (*ColorSensorCallback)(int color, byte port);
typedef void (*DistanceSensorCallback)(int distance, byte port);
typedef void (*ButtonCallback)(ButtonState state);

// Additional command types for thread-safe communication (Main → BLE)
enum CommandType {
    CMD_MOTOR_SPEED = 0, // Base commands from DuploHub
    CMD_STOP_MOTOR,
    CMD_SET_LED_COLOR,
    CMD_SET_HUB_NAME,

    // Extended commands specific to DuploHubExtended
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

class DuploHubExtended : public DuploHub {
private:
    byte motorPort;
    bool wasConnected;  // Track previous connection state

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

    void initFreeRTOS();
    void cleanupFreeRTOS();
    void updateConnectionState(bool connected, bool connecting);
    void processCommandQueue();
    void processSensorData();
    static void bleTaskWrapper(void* parameter);
    void bleTaskFunction();
    static void colorSensorCallbackWrapper(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
    static void distanceSensorCallbackWrapper(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
    static void buttonCallbackWrapper(void *hub, HubPropertyReference hubProperty, uint8_t *pData);

protected:
    void queueSensorData(const SensorData& data);

public:
    DuploHubExtended();
    DuploHubExtended(byte port);
    ~DuploHubExtended();

    void setMotorPort(byte port);
    byte getMotorPort();
    void activateColorSensor(byte port);
    void activateDistanceSensor(byte port);
    void activateButton();
    void setOnConnectedCallback(ConnectionCallback callback);
    void setOnDisconnectedCallback(ConnectionCallback callback);
    void setOnColorSensorCallback(ColorSensorCallback callback);
    void setOnDistanceSensorCallback(DistanceSensorCallback callback);
    void setOnButtonCallback(ButtonCallback callback);
    void update();
};

#endif // DUPLO_HUBEXTENDED_H
