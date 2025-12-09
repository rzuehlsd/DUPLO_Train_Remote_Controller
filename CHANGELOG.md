# Changelog

All notable changes to the DUPLO Train Controller project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.1.0] - 2025-01-20

### Added

- **DuploHubExtended class** with bidirectional sensor data support
- **Sensor callback system** for color, distance, and button events
- **Second FreeRTOS queue** (sensorQueue) for BLE → Main task communication
- **Static callback wrappers** for seamless Lpf2Hub integration
- **Thread-safe sensor activation** via command queue system
- **TrainController_Extended.cpp** demonstrating color-based speed control
- **Real-time sensor processing** with ~50ms latency
- **Emergency distance detection** with automatic motor stopping
- **Button-based manual control** for train operation

### Enhanced
- **Architecture documentation** with bidirectional data flow diagrams
- **API reference** extended with sensor callback examples
- **Performance characteristics** for dual-queue communication system
- **Memory usage analysis** for sensor data processing
- **Complete sensor processing chain analysis** with 8-layer detailed flow
- **Processing timeline documentation** showing 75ms sensor-to-action latency
- **Layer-by-layer code examples** from hardware to application callbacks

### Technical Details
- Sensor data flows BLE Task (Core 0) → Main Task (Core 1) via sensorQueue
- Static callback integration bridges Lpf2Hub callbacks to queue system
- Non-blocking sensor data processing with configurable queue sizes
- Thread-safe sensor activation commands via existing command queue
- Complete backward compatibility with existing DuploHub API

## [2.0.0] - 2025-01-20

### Added
- **Multi-task architecture** with dedicated BLE task running on Core 0
- **Thread-safe command queue system** for inter-task communication
- **FreeRTOS integration** with proper synchronization primitives
- **Automatic connection recovery** with background reconnection logic
- **System health monitoring** with task status checking
- **Professional logging system** with component-specific prefixes
- **TrainController.cpp** as main application (renamed from TrainHub.cpp)
- **DuploHub class** as hardware abstraction layer
- **Comprehensive documentation** including README, Quick Reference, and Architecture guide

### Changed
- **Complete refactoring** from single-threaded to multi-threaded architecture
- **Connection management** moved to background task for non-blocking operation
- **Command execution** now uses queue-based system instead of direct calls
- **Error handling** improved with graceful degradation and recovery
- **Performance optimization** with dual-core ESP32 utilization

### Improved
- **Responsiveness**: Main loop never blocks on BLE operations
- **Reliability**: Automatic task recovery and connection management
- **Maintainability**: Clean separation of concerns between layers
- **Debuggability**: Detailed logging and status monitoring
- **Code quality**: Thread-safe design patterns throughout

## [1.0.0] - 2025-01-19

### Added
- **Initial project structure** with basic DUPLO train control
- **DuploHub class** with basic connection management
- **Connection callbacks** for hub connect/disconnect events
- **Motor control** with speed setting and stop functionality
- **LED control** with color changes
- **Demo sequence** with automatic train operation
- **Serial monitoring** with basic status output

### Technical Details
- Single-threaded architecture using Arduino loop
- Direct Lpf2Hub integration for BLE communication
- Basic connection state tracking
- Simple demo state machine

### Dependencies
- Legoino library for LEGO Powered Up protocol
- NimBLE-Arduino for Bluetooth LE communication
- ESP32 Arduino framework

## [0.1.0] - 2025-01-18 (Initial Development)

### Added
- **Project initialization** with PlatformIO setup
- **Basic Lpf2Hub integration** from Legoino library
- **ESP32 platform configuration**
- **Initial connection attempts** to DUPLO train hub
- **Proof of concept** motor control commands

### Development Notes
- Started as simple example modification
- Explored Legoino library capabilities
- Established basic BLE connectivity
- Verified ESP32 compatibility with LEGO hubs

---

## Upgrade Guide

### From 1.0.0 to 2.0.0

The 2.0.0 release introduces breaking changes due to the multi-task architecture:

#### Required Code Changes

1. **Update setup() function**:
   ```cpp
   // Old (1.0.0)
   void setup() {
       duploHub.init();
   }
   
   // New (2.0.0)
   void setup() {
       duploHub.startBLETask();  // Start background BLE task
   }
   ```

2. **Update main loop**:
   ```cpp
   // Old (1.0.0)
   void loop() {
       if (!duploHub.isConnected()) {
           duploHub.init();
       }
       // ... direct control code
   }
   
   // New (2.0.0)
   void loop() {
       duploHub.update();  // Only handle callbacks
       // ... application logic (no direct BLE calls)
   }
   ```

3. **Thread-safe commands** (automatically handled by backward compatibility):
   ```cpp
   // These calls are now automatically thread-safe
   duploHub.setMotorSpeed(50);
   duploHub.setLedColor(RED);
   ```

#### Benefits After Upgrade
- Non-blocking BLE operations
- Automatic connection recovery
- Better system responsiveness
- Professional status monitoring
- Improved error handling

## Compatibility Matrix

| Version | ESP32 Core | Arduino Framework | PlatformIO | NimBLE | Legoino |
|---------|------------|-------------------|------------|---------|---------|
| 2.0.0   | ≥2.0.0     | ≥2.0.0           | ≥6.0.0     | ≥1.4.0  | ≥1.8.0  |
| 1.0.0   | ≥1.0.6     | ≥1.0.6           | ≥5.0.0     | ≥1.3.0  | ≥1.7.0  |
| 0.1.0   | ≥1.0.4     | ≥1.0.4           | ≥4.0.0     | ≥1.2.0  | ≥1.6.0  |

## Known Issues

### Version 2.0.0
- Initial BLE scan may take up to 10 seconds on first boot
- Memory usage increased due to FreeRTOS objects (~2KB additional)
- Debug output may be verbose during development

### Version 1.0.0
- BLE operations could block main loop for several seconds
- No automatic recovery from connection failures
- Limited error handling and status reporting

## Contributors

- **System Architecture**: Multi-task design and FreeRTOS integration
- **Hardware Abstraction**: DuploHub class design and thread-safe API
- **Documentation**: Comprehensive guides and API documentation
- **Testing**: Hardware validation with DUPLO train systems

## Acknowledgments

- **Cornelius Munz** - Original Legoino library
- **ESP32 Community** - Arduino framework and examples
- **LEGO Group** - DUPLO and Powered Up specifications
