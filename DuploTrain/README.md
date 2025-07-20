# DUPLO Train Controller System

A comprehensive multi-task Arduino application for controlling LEGO DUPLO trains using ESP32 and Bluetooth Low Energy (BLE).

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

### Key Highlights

- **Multi-Task Architecture**: BLE operations run on separate CPU core for optimal performance
- **Thread-Safe Design**: Command queuing system prevents race conditions
- **Automatic Recovery**: Self-healing connection management
- **Real-Time Control**: Low-latency motor and LED control
- **Professional Logging**: Comprehensive system status monitoring

## 🏗️ System Architecture

### Three-Layer Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER                            │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              TrainController.cpp                        │   │
│  │  - Main application logic (Core 1)                     │   │
│  │  - Demo sequence management                             │   │
│  │  - Event handling and callbacks                        │   │
│  │  - User interface and status monitoring                │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────┬───────────────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────────────┐
│              HARDWARE ABSTRACTION LAYER                        │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                  DuploHub.h/.cpp                        │   │
│  │  - Thread-safe API                                      │   │
│  │  - Command queue system                                 │   │
│  │  - BLE task management (Core 0)                        │   │
│  │  - Connection state synchronization                    │   │
│  │  - FreeRTOS integration                                 │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────┬───────────────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────────────┐
│                PROTOCOL LAYER                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              Lpf2Hub (Legoino Library)                  │   │
│  │  - LEGO Powered Up protocol implementation             │   │
│  │  - BLE communication primitives                        │   │
│  │  - Hub-specific command handling                       │   │
│  │  - NimBLE integration                                   │   │
│  └─────────────────────────────────────────────────────────┘   │
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
