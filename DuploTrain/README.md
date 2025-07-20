# DUPLO Train Controller System

A comprehensive multi-task Arduino applicatio### **Multi-Task Processing with Bidirectional Communication**

```
ESP32 DUAL-CORE UTILIZATION:

Core 0 (BLE Task)                    Core 1 (Main Task)
┌─────────────────┐                  ┌─────────────────┐
│ • BLE Operations│                  │ • Application   │
│ • Hub Connection│                  │ • User Interface│
│ • Motor Commands│                  │ • Status Display│
│ • LED Control   │                  │ • Demo Logic    │
│ • Sensor Setup  │                  │ • Sensor Process│
│ • Data Parsing  │                  │ • Callbacks     │
└─────────────────┘                  └─────────────────┘
         ↕                                    ↕
┌─────────────────┐    FreeRTOS     ┌─────────────────┐
│  commandQueue   │ ←──── Queues ───→│  sensorQueue    │
│                 │                  │                 │
│ • Motor Speed   │                  │ • Color Data    │
│ • LED Color     │                  │ • Distance Data │
│ • Sensor Activate│                 │ • Button Events │
│ • Hub Settings  │                  │ • Connection    │
└─────────────────┘                  └─────────────────┘

COMMAND FLOW: Main → BLE (motor control, sensor setup)
SENSOR FLOW:  BLE → Main (sensor data, button presses)
```

**Key Benefits:**
- ✅ **Non-blocking BLE operations**: Main loop never waits
- ✅ **Real-time sensor processing**: ~50ms latency for sensor callbacks
- ✅ **Thread-safe communication**: FreeRTOS queues handle all inter-task data
- ✅ **Automatic connection recovery**: Background reconnection without user intervention
- ✅ **Professional error handling**: Graceful degradation and comprehensive logging LEGO DUPLO trains using ESP32 and Bluetooth Low Energy (BLE).

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Dependencies](#software-dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [API Reference](#api-reference)
- [System Monitoring](#system-monitoring)
- [Troubleshooting](#troubleshooting)
- [Development](#development)
- [Contributing](#contributing)
- [License](#license)

## 🎯 Overview

This project implements a sophisticated train control system that manages LEGO DUPLO trains through a multi-layered, multi-task architecture. The system provides reliable BLE connectivity, automatic connection recovery, and responsive train control while maintaining excellent system performance through dual-core utilization.

### Sensor Issues

**Sensor callbacks not firing:**
- Verify sensor is properly connected to specified port
- Check if `activateColorSensor()` or `activateDistanceSensor()` was called
- Ensure `duploHub.update()` is called regularly in main loop
- Monitor serial output for "BLE Task: Activating * sensor" messages
- Use extended version: `DuploHubExtended duploHub;`

**Sensor data delayed or missing:**
- Check sensor queue status in debug output
- Verify sensor callback registration was successful
- Ensure hub is connected before sensor activation
- Color sensor requires proper lighting conditions
- Distance sensor has ~4cm minimum detection range

**Example sensor debugging:**
```cpp
// Enable detailed sensor logging
void onColorDetected(int color, byte port) {
    Serial.print("Color callback - Color: ");
    Serial.print(LegoinoCommon::ColorStringFromColor(color).c_str());
    Serial.print(", Port: ");
    Serial.println(port);
    
    // Your control logic here
}
```

- **Multi-Task Architecture**: BLE operations run on separate CPU core for optimal performance
- **Thread-Safe Design**: Command queuing system prevents race conditions
- **Automatic Recovery**: Self-healing connection management
- **Real-Time Control**: Low-latency motor and LED control
- **Professional Logging**: Comprehensive system status monitoring

## 🏗️ Architecture Overview

The system uses a **three-layer architecture** with **multi-task processing** and **bidirectional sensor data flow**:

```
┌─────────────────────────────────────────────────────────────────┐
│                    THREE-LAYER ARCHITECTURE                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              LAYER 1: APPLICATION LAYER                    │ │
│ │                   (TrainController.cpp)                    │ │
│ │                                                             │ │
│ │  • Train demo sequence and state management                 │ │
│ │  • User interface and status monitoring                     │ │
│ │  • High-level train control logic                          │ │
│ │  • Connection event handling                                │ │
│ │  • Sensor data processing and callbacks                    │ │
│ └─────────────────────────────────────────────────────────────┘ │
│                   ↕ Clean API + Sensor Callbacks                │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │             LAYER 2: HARDWARE ABSTRACTION                  │ │
│ │                     (DuploHub Class)                       │ │
│ │                                                             │ │
│ │  • Thread-safe command queuing system                      │ │
│ │  • Bidirectional FreeRTOS queues (commands + sensors)      │ │
│ │  • Multi-task management (dual-core ESP32)                 │ │
│ │  • Connection state management                              │ │
│ │  • Automatic recovery and error handling                   │ │
│ │  • Sensor data routing and callback management             │ │
│ └─────────────────────────────────────────────────────────────┘ │
│                   ↕ Protocol Interface + Sensor Integration     │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │              LAYER 3: PROTOCOL LAYER                       │ │
│ │                    (Lpf2Hub Library)                       │ │
│ │                                                             │ │
│ │  • LEGO Powered Up protocol implementation                 │ │
│ │  • Bluetooth LE communication                              │ │
│ │  • Device discovery and connection management              │ │
│ │  • Motor and LED control commands                          │ │
│ │  • Sensor data parsing and callbacks                       │ │
│ └─────────────────────────────────────────────────────────────┘ │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Multi-Task Communication Flow

```
Main Loop (Core 1)              BLE Task (Core 0)
       │                              │
       ▼                              ▼
TrainController.cpp             DuploHub::bleTaskFunction()
       │                              │
       ├─ duploHub.setMotorSpeed() ──►│─ processCommandQueue()
       │                              │
       ├─ duploHub.update() ◄─────────│─ updateConnectionState()
       │                              │
       ├─ Event Callbacks ◄───────────│─ Connection Management
       │                              │
       └─ Demo Logic                  └─ BLE Operations
```

## ✨ Features

### Core Features
- **Multi-Task BLE Management**: Background BLE operations don't block main application
- **Automatic Connection**: Discovers and connects to DUPLO train hubs automatically
- **Auto-Recovery**: Automatically reconnects when connection is lost
- **Thread-Safe Commands**: Queue-based command system prevents race conditions
- **Real-Time Control**: Low-latency motor speed and LED color control

### Advanced Features
- **Dual-Core Utilization**: ESP32 cores optimized for different tasks
- **Connection Callbacks**: Event-driven programming model
- **System Monitoring**: Real-time status reporting and health checks
- **Professional Logging**: Detailed component-specific log messages
- **Resource Management**: Proper FreeRTOS object lifecycle management

### Demo Features
- **Non-Blocking Demo**: Smooth demo sequence that doesn't interfere with connection management
- **Visual Feedback**: LED color changes coordinated with motor movements
- **Safety Features**: Automatic motor stop on disconnection
- **Loop Demo**: Continuous demonstration sequence

## 🔧 Hardware Requirements

### Primary Hardware
- **ESP32 Development Board** (ESP32-DevKitC or compatible)
- **LEGO DUPLO Train Hub** with motor connected to Port A
- **USB Cable** for programming and power

### Supported LEGO Hardware
- DUPLO Train Hub (Hub Type: `DUPLO_TRAIN_HUB`)
- DUPLO Train Motors (connected to Port A)
- Compatible with other Powered Up hubs (with minor modifications)

### Technical Specifications
- **Microcontroller**: ESP32 (dual-core, 240MHz)
- **Memory**: 520KB SRAM, 4MB Flash minimum
- **Connectivity**: Bluetooth 4.2 LE
- **Operating Voltage**: 3.3V
- **Power Consumption**: ~200mA during active operation

## 📦 Software Dependencies

### Platform
- **PlatformIO**: Development platform
- **ESP32 Arduino Framework**: Core ESP32 support
- **FreeRTOS**: Real-time operating system (included with ESP32)

### Libraries
- **Legoino**: LEGO Powered Up protocol implementation
- **NimBLE-Arduino**: Bluetooth Low Energy stack
- **Bounce2**: Button debouncing (part of Legoino dependencies)

### Library Versions
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    legoino/Legoino
    h2zero/NimBLE-Arduino
```

## 🚀 Installation

### Step 1: Environment Setup
```bash
# Install PlatformIO CLI
pip install platformio

# Clone the repository
git clone <repository-url>
cd DuploTrain
```

### Step 2: Project Build
```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Open serial monitor
pio device monitor
```

### Step 3: Hardware Setup
1. Connect ESP32 to computer via USB
2. Power on your DUPLO train hub
3. Ensure motor is connected to Port A of the hub
4. Upload and run the code

## 📖 Usage

### Basic Operation

1. **Startup**: System automatically starts and begins scanning for DUPLO hubs
2. **Connection**: When hub is found, automatic connection is established
3. **Demo**: Once connected, the demo sequence begins automatically
4. **Monitoring**: Status information is displayed every 10 seconds

### Expected Serial Output
```
TrainController: Starting up...
DuploHub: FreeRTOS objects initialized
DuploHub: BLE task started successfully
TrainController: Ready - BLE task running, waiting for hub connection...
BLE Task: Started successfully
BLE Task: Attempting initial connection to hub...
BLE Task: Connected to HUB
TrainController: Hub connected - initializing train demo!
TrainController: Starting train demo sequence...
TrainController Status - BLE Task: Running, Hub Connected: Yes, Demo Active: Yes
```

### Demo Sequence
The system runs a continuous demonstration:
1. **LED Green** → **LED Red** → **Motor Forward** → **Stop** → **Motor Backward** → **Stop** → **Repeat**
2. Each step lasts 1 second
3. Demo stops automatically if hub disconnects
4. Demo resumes automatically when reconnected

## 📚 API Reference

### DuploHub Class

#### Connection Management
```cpp
void startBLETask();                    // Start background BLE task
void stopBLETask();                     // Stop background BLE task
bool isConnected();                     // Check connection status (thread-safe)
bool isConnecting();                    // Check if connecting (thread-safe)
bool isDisconnected();                  // Check if disconnected (thread-safe)
bool isBLETaskRunning();                // Check BLE task status
```

#### Control Methods (Thread-Safe)
```cpp
void setMotorSpeed(int speed);          // Set motor speed (-100 to 100)
void stopMotor();                       // Stop motor immediately
void setLedColor(Color color);          // Set hub LED color
void setHubName(const char* name);      // Set hub name
```

#### Information Methods
```cpp
std::string getHubAddress();            // Get hub BLE address
std::string getHubName();               // Get hub name
byte getMotorPort();                    // Get configured motor port
void setMotorPort(byte port);           // Set motor port
```

#### Event Callbacks
```cpp
void setOnConnectedCallback(ConnectionCallback callback);     // Hub connected event
void setOnDisconnectedCallback(ConnectionCallback callback);  // Hub disconnected event
```

### DuploHubExtended Class (Sensor Support)

#### Sensor Callbacks
```cpp
typedef void (*ColorSensorCallback)(int color, byte port);
typedef void (*DistanceSensorCallback)(int distance, byte port);
typedef void (*ButtonCallback)(ButtonState state);

void setOnColorSensorCallback(ColorSensorCallback callback);      // Color sensor events
void setOnDistanceSensorCallback(DistanceSensorCallback callback);  // Distance sensor events
void setOnButtonCallback(ButtonCallback callback);                 // Button press events
```

#### Sensor Activation (Thread-Safe)
```cpp
void activateColorSensor(byte port);      // Activate color sensor on specified port
void activateDistanceSensor(byte port);   // Activate distance sensor on specified port
void activateButton();                    // Activate hub button monitoring
```

#### Example Usage
```cpp
// Color-based speed control
void onColorDetected(int color, byte port) {
    if (color == (byte)Color::RED) {
        duploHub.stopMotor();           // Emergency stop
    } else if (color == (byte)Color::GREEN) {
        duploHub.setMotorSpeed(50);     // Fast speed
    }
    duploHub.setLedColor((Color)color); // Match LED to detected color
}

// Setup
DuploHubExtended duploHub;
duploHub.setOnColorSensorCallback(onColorDetected);
duploHub.activateColorSensor((byte)PoweredUpHubPort::B);
```

### Available Colors
```cpp
BLACK, PINK, PURPLE, BLUE, LIGHTBLUE, CYAN, GREEN, YELLOW, ORANGE, RED, WHITE
```

### Motor Speed Range
- **Range**: -100 to 100
- **Positive**: Forward direction
- **Negative**: Backward direction
- **Zero**: Stop (equivalent to `stopMotor()`)

## 📊 System Monitoring

### Status Information
The system provides comprehensive monitoring through serial output:

```
TrainController Status - BLE Task: Running, Hub Connected: Yes, Demo Active: Yes
```

### Log Message Prefixes
- **`TrainController:`** - Main application messages
- **`BLE Task:`** - Background BLE task messages  
- **`DuploHub:`** - Hardware abstraction layer messages
- **`ERROR:`** - Critical system errors
- **`WARNING:`** - Non-critical issues

### Performance Monitoring
- **BLE Task**: Runs on Core 0, Priority 2
- **Main Loop**: Runs on Core 1, Priority 1
- **Connection Checks**: Every 1 second
- **Command Processing**: Every 50ms
- **Status Updates**: Every 10 seconds

## 🔄 How Sensor Processing Works

### Complete Processing Flow

When a DUPLO color sensor detects a color change, here's how it flows through the system:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    SENSOR PROCESSING FLOW                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│ 1. DUPLO Sensor    →  2. BLE Radio    →  3. ESP32 Core 0               │
│    Detects RED        Transmits data     NimBLE + Lpf2Hub              │
│                                                                         │
│ 4. Static Callback →  5. Sensor Queue  →  6. ESP32 Core 1              │
│    Parse + Package     FreeRTOS Queue     Main Task Processing         │
│                                                                         │
│ 7. User Callback   →  8. Motor Command →  9. BLE Command               │
│    onColorDetected     stopMotor()         Back to DUPLO Hub           │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘

Total Time: ~75ms from sensor detection to motor response
```

### Key Processing Steps

1. **Hardware Detection** (0ms): DUPLO sensor detects color change
2. **BLE Transmission** (5ms): Sensor data transmitted via Bluetooth
3. **Protocol Parsing** (15ms): Lpf2Hub parses LEGO protocol data
4. **Queue Processing** (20ms): Data packaged and queued between CPU cores
5. **Main Task Processing** (50ms): Main loop processes sensor queue
6. **User Callback** (60ms): Your `onColorDetected()` function is called
7. **Motor Response** (75ms): Train stops/changes speed based on color

### What Makes This Fast & Reliable

- ✅ **Dual-Core Processing**: Sensor processing never blocks motor control
- ✅ **Non-blocking Queues**: No waiting - data flows continuously  
- ✅ **Batch Processing**: Multiple sensor readings processed together
- ✅ **Error Recovery**: System continues working even if sensors fail
- ✅ **Professional Logging**: Every step is logged for debugging

### Example: Color-Based Speed Control
```cpp
void onColorDetected(int color, byte port) {
    // This function is called ~60ms after sensor detects color
    
    if (color == (byte)Color::RED) {
        duploHub.stopMotor();              // Emergency stop
    } else if (color == (byte)Color::GREEN) {
        duploHub.setMotorSpeed(50);        // Fast forward
    }
    
    duploHub.setLedColor((Color)color);    // Visual feedback
}
```

## 🎮 How Motor Control Works

### Complete Command Processing Flow

When you call `duploHub.setMotorSpeed(50)`, here's how it flows through the system:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    COMMAND PROCESSING FLOW                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│ 1. Application     →  2. Thread-Safe   →  3. Command Queue             │
│    setMotorSpeed(50)  Wrapper Function     FreeRTOS Queue              │
│                                                                         │
│ 4. ESP32 Core 1    →  5. ESP32 Core 0   →  6. Protocol Format          │
│    Main Task Queue    BLE Task Processing   Lpf2Hub + LEGO Protocol    │
│                                                                         │
│ 7. BLE Transmission → 8. DUPLO Hub     →  9. Motor Hardware            │
│    NimBLE + Radio      Protocol Parse     PWM Signal to Motor          │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘

Total Time: ~60ms from function call to physical motor response
```

### Key Command Steps

1. **Function Call** (0ms): Application calls `duploHub.setMotorSpeed(50)`
2. **Thread-Safe Wrapping** (1ms): Command packaged for inter-core communication
3. **Queue Processing** (3ms): Command queued from Core 1 to Core 0
4. **BLE Task Processing** (25ms): Core 0 processes command queue (max 50ms)
5. **Protocol Formatting** (35ms): Lpf2Hub formats LEGO Powered Up protocol
6. **BLE Transmission** (45ms): Command sent via Bluetooth to DUPLO hub
7. **Motor Response** (60ms): DUPLO hub applies PWM signal to motor

### What Makes Commands Reliable

- ✅ **Thread-Safe Queuing**: Commands never interfere with sensor processing
- ✅ **Connection Checking**: Commands only execute when hub is connected
- ✅ **Timeout Protection**: 100ms timeout prevents system hangs
- ✅ **Error Recovery**: System continues working if commands fail
- ✅ **Professional Logging**: Every command execution is logged

### Example: Responsive Motor Control
```cpp
void setup() {
    duploHub.startBLETask();  // Start background command processing
}

void loop() {
    duploHub.update();        // Process sensor callbacks
    
    // Commands execute immediately, never block the main loop
    if (emergencyStop) {
        duploHub.stopMotor();           // ~60ms to physical stop
    }
    
    if (speedChange) {
        duploHub.setMotorSpeed(75);     // ~60ms to new speed
    }
}
```

> 📝 **Note**: For complete technical details including code examples and timing analysis, see [ARCHITECTURE.md](ARCHITECTURE.md#complete-sensor-processing-chain-analysis)

## 🔧 Troubleshooting

### Common Issues

#### Connection Problems
**Symptom**: "BLE Task: Attempting initial connection..." but never connects

**Solutions**:
1. Ensure DUPLO hub is powered on and in pairing mode
2. Check if hub is already connected to another device
3. Reset hub by holding button for 10 seconds
4. Move ESP32 closer to hub (within 5 meters)

#### Task Failures
**Symptom**: "ERROR: Failed to start BLE task"

**Solutions**:
1. Check available heap memory: `ESP.getFreeHeap()`
2. Reduce other running tasks
3. Increase stack size in `startBLETask()`

#### Command Delays
**Symptom**: Motor/LED commands seem delayed

**Solutions**:
1. Check command queue status
2. Verify BLE connection stability
3. Reduce command frequency in application

### Debug Mode
Enable detailed debugging by adding to `platformio.ini`:
```ini
build_flags = -DCORE_DEBUG_LEVEL=4
monitor_filters = esp32_exception_decoder
```

## 🛠️ Development

### Project Structure
```
DuploTrain/
├── include/
│   └── DuploHub.h              # Hardware abstraction layer header
├── src/
│   ├── TrainController.cpp     # Main application
│   └── DuploHub.cpp           # Hardware abstraction implementation  
├── lib/
│   ├── Legoino/               # LEGO Powered Up protocol library
│   ├── NimBLE-Arduino/        # Bluetooth LE stack
│   └── Bounce2/               # Button handling library
├── platformio.ini             # Build configuration
└── README.md                  # This file
```

### Adding New Features

#### New Train Commands
1. Add command type to `CommandType` enum in `DuploHub.h`
2. Add command data structure to `HubCommand` union
3. Implement thread-safe method in `DuploHub` class
4. Add processing logic to `processCommandQueue()`

#### New Hub Types
1. Modify hub type detection in BLE scanning
2. Add hub-specific command handling
3. Update motor port configurations if needed

### Code Style Guidelines
- Use clear, descriptive variable names
- Add comprehensive comments for complex logic
- Follow Arduino coding standards
- Use thread-safe practices for shared data
- Include error handling for all operations

## 🤝 Contributing

### Development Process
1. Fork the repository
2. Create feature branch (`git checkout -b feature/new-feature`)
3. Follow code style guidelines
4. Add comprehensive comments
5. Test thoroughly on hardware
6. Submit pull request with detailed description

### Testing Checklist
- [ ] Builds without warnings
- [ ] Connects to DUPLO hub successfully
- [ ] Motor control works in both directions
- [ ] LED colors change correctly
- [ ] Reconnection works after power cycle
- [ ] No memory leaks during long operation
- [ ] Task monitoring shows healthy system

## 📄 License

MIT License - see LICENSE file for details

## 🙏 Acknowledgments

- **Cornelius Munz** - Legoino library creator
- **LEGO Group** - DUPLO and Powered Up hardware
- **Espressif** - ESP32 platform
- **Arduino Community** - Framework and libraries

---

**Version**: 2.0.0  
**Last Updated**: January 2025  
**Compatibility**: ESP32, Arduino Framework, PlatformIO
