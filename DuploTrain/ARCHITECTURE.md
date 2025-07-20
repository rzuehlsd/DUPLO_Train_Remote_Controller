# Technical Architecture Document

## System Overview

The DUPLO Train Controller implements a sophisticated three-layer, multi-task architecture optimized for ESP32 dual-core systems.

## Multi-Task Architecture

### Core Allocation Strategy

```
ESP32 Core 0 (Protocol Core)        ESP32 Core 1 (Application Core)
├── BLE Stack (NimBLE)              ├── Arduino Main Loop
├── BLE Task (DuploHub)             ├── TrainController Logic  
├── FreeRTOS Scheduler              ├── Demo State Machine
└── Hardware Interrupts             └── User Callbacks
```

### Task Priorities & Timing

| Task | Core | Priority | Stack Size | Timing |
|------|------|----------|------------|--------|
| Arduino Loop | 1 | 1 | 8KB | Continuous |
| BLE Task | 0 | 2 | 4KB | 50ms cycle |
| Idle Tasks | Both | 0 | 1KB | As needed |

## Thread Synchronization

### Synchronization Primitives

```cpp
class DuploHub {
private:
    SemaphoreHandle_t connectionMutex;    // Protects connection state
    QueueHandle_t commandQueue;           // Inter-task communication
    TaskHandle_t bleTaskHandle;           // Task lifecycle management
    volatile bool connectionState;        // Atomic state variables
    volatile bool connectingState;
};
```

### Critical Sections

```cpp
// Example: Thread-safe connection state access
bool DuploHub::isConnected() {
    xSemaphoreTake(connectionMutex, portMAX_DELAY);
    bool state = connectionState;
    xSemaphoreGive(connectionMutex);
    return state;
}
```

### Command Queue Protocol

```cpp
// Command structure for inter-task communication
typedef struct {
    CommandType type;           // Command identifier
    union {
        struct { int speed; } motor;
        struct { Color color; } led;
        struct { char name[32]; } hubName;
    } data;
} HubCommand;
```

## Memory Management

### Static Allocation Strategy
- **Command Queue**: 10 × `sizeof(HubCommand)` = ~320 bytes
- **Task Stacks**: BLE Task (4KB), Main Loop (8KB)
- **Synchronization Objects**: ~100 bytes total
- **String Buffers**: Fixed-size arrays for thread safety

### Heap Usage Optimization
- Minimal dynamic allocation
- String operations use stack buffers
- BLE operations handled by NimBLE stack
- No memory leaks in main application code

## Communication Flow

### Startup Sequence

```mermaid
sequenceDiagram
    participant App as TrainController
    participant Hub as DuploHub
    participant BLE as BLE Task
    participant LEGO as Lpf2Hub

    App->>Hub: Constructor (initFreeRTOS)
    Hub->>Hub: Create Mutex & Queue
    App->>Hub: startBLETask()
    Hub->>BLE: Create Task (Core 0)
    BLE->>LEGO: hub.init() - Start scanning
    LEGO-->>BLE: Device found callback
    BLE->>LEGO: hub.connectHub()
    BLE->>Hub: updateConnectionState(true)
    Hub->>App: onConnectedCallback()
```

### Command Execution Flow

```mermaid
sequenceDiagram
    participant App as TrainController
    participant Queue as Command Queue
    participant BLE as BLE Task
    participant LEGO as Lpf2Hub

    App->>Queue: setMotorSpeed(50)
    Queue-->>BLE: Command available
    BLE->>Queue: Dequeue command
    BLE->>LEGO: setBasicMotorSpeed(port, 50)
    LEGO-->>BLE: BLE transmission complete
```

## Error Handling Strategy

### Failure Recovery Mechanisms

1. **BLE Task Monitoring**: Main loop checks task health every 5 seconds
2. **Connection Recovery**: Automatic reconnection with exponential backoff
3. **Command Queue Overflow**: Non-blocking queue operations with warnings
4. **Resource Cleanup**: Proper destruction in destructors and error paths

### Error Categories

| Category | Handling | Recovery |
|----------|----------|----------|
| BLE Connection Loss | Automatic retry | Background reconnection |
| Task Creation Failure | Error logging | Fallback to synchronous mode |
| Queue Full | Drop command + warning | Continue operation |
| Memory Allocation | Error logging | Graceful degradation |

## Performance Characteristics

### Latency Analysis

| Operation | Typical Latency | Maximum Latency |
|-----------|----------------|-----------------|
| Motor Command | 50-100ms | 200ms |
| LED Command | 30-80ms | 150ms |
| Connection Detection | 1-3 seconds | 10 seconds |
| State Query | <1ms | 5ms (mutex wait) |

### Throughput Capabilities

- **Command Rate**: ~20 commands/second sustainable
- **BLE Bandwidth**: ~1KB/second LEGO protocol data
- **CPU Utilization**: ~15% Core 0, ~5% Core 1 during active operation

## Scalability Considerations

### Adding New Hub Types

```cpp
// Extend command types
enum CommandType {
    CMD_MOTOR_SPEED,
    CMD_STOP_MOTOR,
    CMD_SET_LED_COLOR,
    CMD_NEW_DEVICE_TYPE    // Add new commands here
};

// Extend command data
typedef struct {
    CommandType type;
    union {
        // Existing commands...
        struct { 
            byte devicePort;
            int deviceValue;
        } newDevice;
    } data;
} HubCommand;
```

### Multiple Hub Support

The architecture can be extended to support multiple hubs:

```cpp
class MultiHubController {
private:
    std::vector<DuploHub*> hubs;
    TaskHandle_t coordinatorTask;
    
public:
    void addHub(DuploHub* hub);
    void broadcastCommand(HubCommand cmd);
    void sendToHub(int hubId, HubCommand cmd);
};
```

## Debug and Monitoring Infrastructure

### Logging Levels

```cpp
#define LOG_LEVEL_ERROR   1
#define LOG_LEVEL_WARNING 2  
#define LOG_LEVEL_INFO    3
#define LOG_LEVEL_DEBUG   4

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#else
#define DEBUG_PRINT(x)
#endif
```

### Runtime Statistics

```cpp
// Available system monitoring
void printSystemStats() {
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("BLE task stack high water mark: %d\n", 
                  uxTaskGetStackHighWaterMark(bleTaskHandle));
    Serial.printf("Queue messages waiting: %d\n", 
                  uxQueueMessagesWaiting(commandQueue));
}
```

## Security Considerations

### BLE Security
- No pairing/bonding implemented (LEGO hubs don't require it)
- Connection based on service UUID matching
- Limited to LEGO protocol commands only

### Code Security
- No dynamic code execution
- Fixed-size buffers prevent overflows
- Input validation on all external data

## Future Enhancement Opportunities

### Planned Improvements
1. **Web Interface**: HTTP server for remote control
2. **Sensor Integration**: Support for LEGO sensors
3. **Multiple Trains**: Coordinate multiple train hubs
4. **Path Planning**: Automated route following
5. **Voice Control**: Integration with speech recognition

### Architecture Extensions
- **Plugin System**: Loadable modules for different train behaviors
- **Event System**: Pub/sub pattern for loose coupling
- **Configuration Management**: Persistent settings storage
- **Over-the-Air Updates**: Remote firmware updates

---

**Document Version**: 1.0  
**Last Updated**: January 2025  
**Target Audience**: Advanced developers and system architects
