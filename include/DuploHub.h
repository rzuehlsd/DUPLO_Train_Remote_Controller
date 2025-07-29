/**
 * DuploHub - A class to encapsulate LEGO DUPLO Train Hub functionality
 * 
 * (c) Copyright 2025
 * Released under MIT License
 * 
 */

#ifndef DUPLO_HUB_H
#define DUPLO_HUB_H

#include "myLegoHub.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"




namespace DuploEnums {

    enum struct DuploTrainDeviceType
    {
        DUPLO_TRAIN_BASE_MOTOR = 41,
        DUPLO_TRAIN_BASE_SPEAKER = 42,
        DUPLO_TRAIN_BASE_COLOR_SENSOR = 43,
        DUPLO_TRAIN_BASE_SPEEDOMETER = 44,
        VOLTAGE_SENSOR = 20,
        LIGHT = 8
    };

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
    enum CommandType {
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

    // Enum for response types
    enum ResponseType {
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

// Response structure for queue communication
typedef struct {
    DuploEnums::ResponseType type;
    union {
        struct {
            DuploEnums::DuploColor detectedColor;
        } colorResponse;
        struct {
            int detectedSpeed;
        } speedResponse;
        struct {
            float detectedVoltage;
        } voltageResponse;
        // Add other response types here as needed
    } data;
} HubResponse;


// Threshold definition for callback execution
#define VOLTAGE_THRESHOLD  0.1f 
#define SPEED_THRESHOLD  5


// DuploHub class definition
// This class encapsulates the functionality of the LEGO DUPLO Train Hub
// It provides methods for connecting to the hub, controlling motors, lights, and sensors,
// and handling BLE communication in a thread-safe manner.
// The class uses FreeRTOS for task management and synchronization. 

class DuploHub {
private:
    myLegoHub hub;
    byte motorPort;
    bool wasConnected;  // Track previous connection state
    
    // Static instance pointer for callback access
    static DuploHub* instance;
    
    // Thread-safe state variables
    volatile bool connectionState;
    volatile bool connectingState;
    
    // FreeRTOS synchronization objects
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
    
    // Private methods for task management
    void initFreeRTOS();
    void cleanupFreeRTOS();
    void updateConnectionState(bool connected, bool connecting);
    void processCommandQueue();
    void clearQueues();
    static void bleTaskWrapper(void* parameter);
    void bleTaskFunction();
    



protected:
      
    // Add declaration for colorSensorCallback to be registered with lpf2hub
    typedef void (*ColorSensorCallback)(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
    static void staticColorSensorCallback(void* hub, byte portNumber, DeviceType deviceType, uint8_t* pData);
    
    // Add declaration for speedSensorCallback to be registered with lpf2hub
    typedef void (*SpeedSensorCallback)(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
    static void staticSpeedSensorCallback(void* hub, byte portNumber, DeviceType deviceType, uint8_t* pData);
    
    // Add declaration for voltageSensorCallback to be registered with lpf2hub
    typedef void (*VoltageSensorCallback)(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData);
    static void staticVoltageSensorCallback(void* hub, byte portNumber, DeviceType deviceType, uint8_t* pData);
    
    
public:
    // Constructor & Destructor
    DuploHub();
    DuploHub(byte port);
    ~DuploHub();  // Destructor for cleanup
    
    // Initialization and connection management
    void init();
    void init(const std::string& address);

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
    void setHubName(const char* name);
    std::string getHubAddress();
    std::string getHubName();
    void listDevicePorts();
    
    // Motor control
    void setMotorPort(byte port);
    byte getMotorPort();
    void setMotorSpeed(int speed);
    void stopMotor();
    
    // Sound control
    void activateBaseSpeaker(); 
    void playSound(int soundId);

    // Light control
    void activateRgbLight();
    void setLedColor(DuploEnums::DuploColor color);
    
    // Direct DUPLO-specific implementations (low-level)
    void playSoundDirect(byte sound);
    void setLedColorDirect(Color color);
    
    // port activation methods
    void activateBaseSpeakerDirect();
    void activateRgbLightDirect();
    void activateColorSensor(); // Legacy method for backward compatibility
    void activateSpeedSensor(); // Legacy method for backward compatibility
    void activateVoltageSensor(); // Voltage sensor activation
   
    // Response Queue Callback registration
    void setOnConnectedCallback(ConnectionCallback callback);
    void setOnDisconnectedCallback(ConnectionCallback callback);
    void setDetectedColorCallback(DetectedColorCallback callback);
    void setDetectedSpeedCallback(DetectedSpeedCallback callback);
    void setDetectedVoltageCallback(DetectedVoltageCallback callback);

    // Process response queue events in main task
    void processResponseQueue();
    



    // BLE Task Function executed on second core
    void update();
};

#endif // DUPLO_HUB_H
