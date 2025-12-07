# DUPLO Train Remote Controller System

**A event-driven train remote control system for LEGO DUPLO, built on ESP32, FreeRTOS, and Bluetooth Low Energy (BLE).**

---

**Author:** Ralf Zühlsdorff  
**Copyright:** 2022-2025 Ralf Zühlsdorff

**Version:** 2.0.0  
**Last Updated:** July 2025  
**Compatibility:** ESP32, Arduino Framework, PlatformIO

**Disclaimer:** LEGO® and Duplo® are trademarks of the LEGO Group of companies which does not sponsor, authorize or endorse this project.

**License:** MIT License (see LICENSE file for details)

---


## Table of Contents

- [Project Overview](#-project-overview)
- [Architecture Overview](#-architecture-overview)
- [Dual-Core Strategy](#-dual-core-strategy)
- [System Architecture](#-system-architecture)
- [Features](#-features)
- [Hardware Requirements](#hardware-requirements)
- [Hardware Roadmap](#-hardware-roadmap)
- [Software Dependencies](#software-dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [User Interaction & LED Feedback](#-user-interaction--led-feedback)
- [API Reference](#api-reference)
- [System Monitoring](#system-monitoring)
- [Troubleshooting](#troubleshooting)
- [Development](#development)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)
- [Architecture](ARCHITECTURE.md)
- [Changelog](CHANGELOG.md)

---

## Project Overview

This project implements a robust, multi-layered, event-driven train controller for LEGO DUPLO trains. The firmware builds on top of the open-source [Legoino](https://github.com/corneliusmunz/legoino) library created by Cornelius Munz, reusing its Powered Up protocol support while adding a hardened hardware abstraction tailored to ESP32 dual-core devices.

Key pillars of the solution include:

- **ESP32 dual-core architecture** (FreeRTOS): BLE communication runs independently from application logic for deterministic responsiveness.
- **Legoino-powered protocol stack**: Proven LEGO Powered Up discovery, device activation, and message parsing.
- **Thread-safe command and sensor queues**: Bidirectional, non-blocking communication between BLE and main application tasks.
- **Automatic connection management**: Self-healing BLE connections with exponential backoff recovery.
- **Professional logging and monitoring**: Real-time status, error, and performance reporting.

The high-level structure is summarised below; a deeper dive is available in [ARCHITECTURE.md](ARCHITECTURE.md).

---

## User Interface

### Quick Start: Connecting a DUPLO Train
- Power on the controller (ESP32) and the DUPLO Train Hub. 
- The DUPLO train will show a **white** blinking front light.
- The RGB LED of the controller turns to **yellow**.
- The controller immediately begins scanning for an active duplo train.
- Once internal initializatioon of the controller is completed the RBB LED turns to **green**. 
- If connected the DUPLO train will show a solid **blue** front light.
- After DUPLO train has connected, the controller’s RGB status LED turns to *blue** and blinks 3 times to confirm link.
- If no hub is found, the status LED pulses **orange** every few seconds to prompt you to power cycle or move the train closer.

### Primary Controls
| Control | Action | Behaviour |
|---------|--------|-----------|
| Rotary encoder (turn) | Adjust train speed | Smooth throttle control from stop to full speed forward or backward with a centre detent for effortless neutral. |
| Rotary encoder (press) | Wake controller / confirm prompts | Single press wakes the unit from deep sleep. |
| Record button | Capture current driving session | RGB LED blinks slowly **red** while recording and stores timing for later playback. Push button again to stop recording.|
| Play button | Replay last recorded session | RGB LED  blinks slowly **yellow** during replay; train reproduces motor, light, and sound cues. Press button again to stop replay of recorded session.|
| Stop button | Immediate emergency stop | Motor power cuts instantly; RGB LED switches to solid **red** and plays break sound. Press button again to resume. RGB LED will go off and departure sound will be played.|
| Light button | Cycle hub LED colours | Steps through preset colours so kids can theme their train without menus. |
| Sound button | Trigger sound effects | Plays horn, brake, or station sounds instantly for interactive storytelling. |
| Water button | Start refill routine | Pauses the train, plays refill sound, and resumes the previous throttle setting. |

### Optical Feedback (Controller RGB LED)
- **Blue (pulsing 5 times):** Connection to a DUPLO hub establish and ready for commands.
- **Red (slow blinking):** Recording action sequence.
- **Yellow (slow blinking):** Replaying action sequence.
- **Red (steady):** Emergency stop engaged. Goes off if button is engaged again
- **Red (fast pulses 5 times):** Signals low voltage of DUPLO train batterie every 30 seconds

### Auto-Sleep & Wake-Up
- After **5 minutes** without button presses or encoder turns, the controller enters deep sleep to save power; the status LED fades out to indicate standby.
- To reactivate, press the rotary encoder once to start initialization and connection cycle again


---
## Architecture Overview

The firmware is organised into three cooperating layers:

1. **Application Layer (`TrainController`)** – orchestrates user input, status LED feedback, replay/record logic, and system health checks.
2. **Hardware Abstraction Layer (`DuploHub`)** – maintains thread-safe command/response queues, sensors, and lifecycle management for the BLE task.
3. **Protocol Layer (`myLegoHub`/Legoino)** – bridges to the LEGO Powered Up protocol, handling device discovery and low-level message encoding.

These layers communicate exclusively through FreeRTOS queues and callbacks, which allows the BLE task and user interface to stay decoupled. The full architectural description, sequence diagrams, and performance considerations are documented in [ARCHITECTURE.md](ARCHITECTURE.md).

## Dual-Core Strategy

The ESP32’s two cores are used deliberately to isolate time-critical communication from user-driven workloads:

- **Core 0** is dedicated to the BLE stack (NimBLE) and the `DuploHub` task, guaranteeing consistent connection upkeep and sensor polling under tight timing constraints.
- **Core 1** runs the Arduino loop (`TrainController`), handling rotary encoder updates, button ladders, response queue processing, and visual/audible feedback without blocking the BLE stack.

This separation keeps latency predictable—even when the UI is busy—and prevents sensor interrupts or sound/LED routines from starving the communication pipeline.

## Future Improvements

- Small Duplo Train Controller based on ESP32-S3 with integrated batterie management
- small Lithium Batterie
- Instead of Poti use of rotary encoder
- Function to record actions and play back (Buttons Rec, Play)
- small Duplo sized printed housing
- only 3 Buttons to control LED, sound and emergency stop
- Function to pair with train (store ID and use when connecting)


---
## System Architecture

### Multi-Task Processing with Bidirectional Communication

```mermaid
graph LR
   MainTask["<b>Core 1 – Main Task</b><br/>• Application logic<br/>• User interface<br/>• Status display<br/>• Demo logic<br/>• Sensor processing<br/>• Event callbacks"]

   BleTask["<b>Core 0 – BLE Task</b><br/>• BLE operations<br/>• Hub connection<br/>• Motor commands<br/>• LED control<br/>• Sensor setup<br/>• Data parsing"]

   MainTask --> |CommandQueue| BleTask

   BleTask --> |SensorQueue| MainTask

   classDef queueStyle fill:#f2f2f2,stroke:#333,stroke-width:1px
   class CommandQueue,SensorQueue queueStyle
```





### Three-Layer Architecture

```mermaid
graph LR
   Layer1["<b>Layer 1: Application Layer</b><br/>• Train demo sequence &amp; state management<br/>• User interface &amp; status monitoring<br/>• High-level control logic<br/>• Connection event handling<br/>• Sensor data processing &amp; callbacks"]

   Layer2["<b>Layer 2: Hardware Abstraction</b><br/>• Thread-safe command queue<br/>• Bidirectional FreeRTOS queues<br/>• Multi-task management (dual-core ESP32)<br/>• Connection state management<br/>• Automatic recovery &amp; error handling<br/>• Sensor routing &amp; callback management"]

   Layer3["<b>Layer 3: Protocol Layer</b><br/>• LEGO Powered Up protocol<br/>• Bluetooth LE communication<br/>• Device discovery &amp; connection<br/>• Motor &amp; LED control commands<br/>• Sensor data parsing &amp; callbacks"]

   Layer1 -->|Clean API + sensor callbacks| Layer2
   Layer2 -->|Protocol interface + sensor integration| Layer3
```

### duploHub.init() Bootstrapping Flow

The following sequence diagram visualizes how the application brings up the BLE infrastructure when `duploHub.init()` is called from `TrainController::setup()`.

```mermaid
sequenceDiagram
   participant TC as TrainController::setup
   participant DH as DuploHub
   participant RTOS as FreeRTOS Scheduler
   participant BLE as BLE Task (Core 0)
   participant Lpf2 as Lpf2Hub

   TC->>DH: init()
   activate DH
   DH->>DH: initFreeRTOS()
   DH->>RTOS: xTaskCreatePinnedToCore(bleTaskWrapper)
   RTOS-->>DH: Task handle
   deactivate DH
   DH-->>TC: return

   RTOS->>BLE: schedule bleTaskWrapper(this)
   activate BLE
   BLE->>DH: bleTaskFunction()
   loop continuous loop
      BLE->>DH: updateBLE()
      alt Hub disconnected
         DH->>Lpf2: hub.init()
      else Hub connecting
         DH->>Lpf2: connectHub()
      end
      opt Connection state change
         DH-->>TC: onConnectedCallback()/onDisconnectedCallback()
      end
      BLE->>DH: processCommandQueue()
      DH->>Lpf2: Execute queued command
      BLE->>BLE: vTaskDelay(50 ms)
   end
```

---

## Features

- **Multi-Task BLE Management**: BLE operations run on a dedicated core, never blocking the main application.
- **Automatic Connection & Recovery**: Discovers, connects, and reconnects to DUPLO train hubs automatically.
- **Thread-Safe Commands**: Queue-based command system prevents race conditions.
- **Real-Time Control**: Low-latency motor speed and LED color control driven by rotary encoder input.
- **Sensor Integration**: Color, speed, and voltage sensors with callback support and queue isolation.
- **Color Stability Filtering**: Timer-based debounce suppresses transient color readings before they reach the UI.
- **Command Recording & Replay**: Capture user sessions and play them back with preserved timing.
- **Professional Logging**: Detailed, component-specific log messages and system status monitoring.
- **Idle Power Management**: Automatic deep sleep after configurable idle periods, resume via encoder button.
- **Resource Management**: Proper FreeRTOS object lifecycle management.
- **Input Abstraction**: ADC button ladder and rotary encoder handlers with debounced callbacks.
- **Demo Mode**: Non-blocking demo sequence with visual feedback and safety features.

---

## Hardware Requirements

### Primary Hardware

- **ESP32 Development Board** (ESP32-S3 Dev kit or compatible)
    I use the Tenstar ESP32-S3 FH4R2 SuperMini which also includes cuircits to charge a 3.7V Lithium battery.
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

---

## Hardware Roadmap

- A dedicated ESP32-S3 carrier board with battery management and button/encoder headers is being finalised in KiCad. The design files have be added to this repository in directory **hardware**.
- A matching printable enclosure is being modelled in OpenSCAD to fit DUPLO geometry and house the controller, battery, and user controls. The `.scad` source will accompany the board files when released.
- Until the hardware package lands, the firmware runs on off-the-shelf ESP32 DevKit boards with external wiring as described above.

---

## Software Dependencies

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

---

## Installation

### 1. Environment Setup

```bash
# Install PlatformIO CLI
pip install platformio

# Clone the repository
git clone <repository-url>
cd DuploTrain
```

### 2. Project Build & Upload

```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Open serial monitor
pio device monitor
```

### 3. Hardware Setup

1. Connect ESP32 to computer via USB
2. Power on your DUPLO train hub
3. Ensure motor is connected to Port A of the hub
4. Upload and run the code

---

## Usage

### Basic Operation

1. **Startup**: System automatically starts and begins scanning for DUPLO hubs
2. **Connection**: When hub is found, automatic connection is established
3. **Demo**: Once connected, the demo sequence begins automatically
4. **Monitoring**: Status information is displayed every 10 seconds

### Expected Serial Output

```text
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

---

## User Interaction & LED Feedback

| Interaction | Description | Hub Response | Status LED Feedback |
|-------------|-------------|--------------|---------------------|
| Rotary encoder turn | Adjusts desired train speed in ±5 steps | Issues `setMotorSpeed()` with normalized value or stops within the dead zone | None (hub LED reflects chosen colour elsewhere) |
| Rotary encoder press | Logged when connected (reserved for future shortcuts) | Diagnostic log entry only | None |
| Record button | Toggles command capture session | Starts/stops `DuploHub::recordCommands()` and clears replay flags | Blinks **blue** while recording, off when idle |
| Play button | Toggles replay of the last recorded session | Invokes `DuploHub::replayCommands()` / `stopReplay()` | Blinks **yellow** during playback, off when idle |
| Stop button | Acts as emergency stop or cancels replay | Stops the motor, plays brake sound, resets replay state | Solid **red** when emergency stop active, off otherwise |
| Light button | Cycles DUPLO hub LED colours | Sends `setLedColor()` with the next palette entry | Hub LED changes colour (status LED unchanged) |
| Sound button | Steps through horn / brake / station samples | Calls `playSound()` with the next sound ID (skips refill) | None |
| Water button | Triggers the refill routine | Plays water refill sound and prepares LED transitions | None |
| Idle timeout (~30 s) | No user input for the configured window | Calls `esp_deep_sleep_start()` | Brief status flash before sleep |

Colour sensor events provide additional automation:

- **Red tile** toggles the emergency stop, forces the hub LED red, blinks the status LED red, and plays the brake sound.
- **Yellow tile** plays the horn while pulsing both LEDs yellow.
- **Blue tile** pauses the train, plays the refill sound, then restores the last commanded speed.
- **White tile** flashes both LEDs white three times as a checkpoint signal.
- **Black tile** restores a neutral state by setting both LEDs to black.

These interaction rules are processed via `DuploHub::processResponseQueue()` using the colour stability timer described in [ARCHITECTURE.md](ARCHITECTURE.md).

---

## API Reference

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

---

## System Monitoring

- **BLE Task**: Runs on Core 0, Priority 2
- **Main Loop**: Runs on Core 1, Priority 1
- **Connection Checks**: Every 1 second
- **Command Processing**: Every 50ms
- **Status Updates**: Every 10 seconds

### Status Information

The system provides comprehensive monitoring through serial output:

```text
TrainController Status - BLE Task: Running, Hub Connected: Yes, Demo Active: Yes
```

### Log Message Prefixes

- **`TrainController:`** - Main application messages
- **`BLE Task:`** - Background BLE task messages  
- **`DuploHub:`** - Hardware abstraction layer messages
- **`ERROR:`** - Critical system errors
- **`WARNING:`** - Non-critical issues

---

## How Sensor Processing Works

When a DUPLO color sensor detects a color change, here's how it flows through the system:

```text
1. DUPLO Sensor    →  2. BLE Radio    →  3. ESP32 Core 0
   Detects RED        Transmits data     NimBLE + Lpf2Hub
4. Static Callback →  5. Sensor Queue  →  6. ESP32 Core 1
   Parse + Package     FreeRTOS Queue     Main Task Processing
7. User Callback   →  8. Motor Command →  9. BLE Command
   onColorDetected     stopMotor()         Back to DUPLO Hub
```

- **Total Time:** ~75ms from sensor detection to motor response

---

## How Motor Control Works

When you call `duploHub.setMotorSpeed(50)`, here's how it flows through the system:

```text
1. Application     →  2. Thread-Safe   →  3. Command Queue
   setMotorSpeed(50)  Wrapper Function     FreeRTOS Queue
4. ESP32 Core 1    →  5. ESP32 Core 0   →  6. Protocol Format
   Main Task Queue    BLE Task Processing   Lpf2Hub + LEGO Protocol
7. BLE Transmission → 8. DUPLO Hub     →  9. Motor Hardware
   NimBLE + Radio      Protocol Parse     PWM Signal to Motor
```

- **Total Time:** ~60ms from function call to physical motor response

---

## Troubleshooting

### Common Issues & Solutions

#### Connection Problems
- Ensure DUPLO hub is powered on and in pairing mode
- Check if hub is already connected to another device
- Reset hub by holding button for 10 seconds
- Move ESP32 closer to hub (within 5 meters)

#### Task Failures
- Check available heap memory: `ESP.getFreeHeap()`
- Reduce other running tasks
- Increase stack size in `startBLETask()`

#### Command Delays
- Check command queue status
- Verify BLE connection stability
- Reduce command frequency in application

### Debug Mode

Enable detailed debugging by adding to `platformio.ini`:

```ini
build_flags = -DCORE_DEBUG_LEVEL=4
monitor_filters = esp32_exception_decoder
```

---

## Development

### Project Structure

```text
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

---

## Contributing

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

---

## License

MIT License - see LICENSE file for details

---

## Acknowledgments

- **Cornelius Munz** - Legoino library creator
- **LEGO Group** - DUPLO and Powered Up hardware
- **Espressif** - ESP32 platform
- **Arduino Community** - Framework and libraries

---

**Version:** 2.0.0  
**Last Updated:** July 2025  
**Compatibility:** ESP32, Arduino Framework, PlatformIO
