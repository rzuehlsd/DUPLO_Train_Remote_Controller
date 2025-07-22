/**
 * DuploHub - A class to encapsulate LEGO DUPLO Train Hub functionality
 * 
 * (c) Copyright 2025
 * Released under MIT License
 * 
 */

#ifndef DUPLO_HUB_H
#define DUPLO_HUB_H

#include "Lpf2Hub.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// Callback function types
typedef void (*ConnectionCallback)();

namespace DuploEnums {
    // Command types for thread-safe communication
    enum CommandType {
        CMD_MOTOR_SPEED,
        CMD_STOP_MOTOR,
        CMD_SET_LED_COLOR,
        CMD_SET_HUB_NAME,
        CMD_PLAY_SOUND
    };

    // Enum for available Duplo sounds
    enum DuploSound {
        BRAKE = 3,
        STATION_DEPARTURE = 5,
        WATER_REFILL = 7,
        HORN = 9,
        STEAM = 10
    };

    // Enum for available Duplo colors
    enum DuploColor {
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
}

// Command structure for queue communication
typedef struct {
    DuploEnums::CommandType type;
    union {
        struct {
            int speed;
        } motor;
        struct {
            DuploEnums::DuploColor color;
        } led;
        struct {
            char name[32];  // Fixed size for thread safety
        } hubName;
        struct {
            int soundId;
        } sound;
    } data;
} HubCommand;

class DuploHub {
private:
    Lpf2Hub hub;
    byte motorPort;
    bool wasConnected;  // Track previous connection state
    
    // Thread-safe state variables
    volatile bool connectionState;
    volatile bool connectingState;
    
    // FreeRTOS synchronization objects
    SemaphoreHandle_t connectionMutex;
    QueueHandle_t commandQueue;
    TaskHandle_t bleTaskHandle;
    
    // Callback functions
    ConnectionCallback onConnectedCallback;
    ConnectionCallback onDisconnectedCallback;
    
    // Private methods for task management
    void initFreeRTOS();
    void cleanupFreeRTOS();
    void updateConnectionState(bool connected, bool connecting);
    void processCommandQueue();
    static void bleTaskWrapper(void* parameter);
    void bleTaskFunction();

protected:
    // Thread-safe implementation methods (used internally)
    void setHubName_ThreadSafe(const char* name);
    void setLedColor_ThreadSafe(DuploEnums::DuploColor color);
    void setMotorSpeed_ThreadSafe(int speed);
    void stopMotor_ThreadSafe();
    void playSound_ThreadSafe(int soundId);
    
public:
    // Constructor & Destructor
    DuploHub();
    DuploHub(byte port);
    ~DuploHub();  // Destructor for cleanup
    
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
    
    // Legacy methods (for backward compatibility)
    void setHubName(const char* name) { setHubName_ThreadSafe(name); }
    void setLedColor(DuploEnums::DuploColor color)  { setLedColor_ThreadSafe(color); }
    void setMotorSpeed(int speed) { setMotorSpeed_ThreadSafe(speed); }
    void stopMotor() { stopMotor_ThreadSafe(); }
    void playSound(int soundId) { playSound_ThreadSafe(soundId); };
    
    // Callback registration
    void setOnConnectedCallback(ConnectionCallback callback);
    void setOnDisconnectedCallback(ConnectionCallback callback);
    
    // Main update loop
    void update();
};

#endif // DUPLO_HUB_H
