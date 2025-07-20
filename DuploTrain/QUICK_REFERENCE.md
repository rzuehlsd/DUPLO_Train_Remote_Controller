# Quick Reference Guide

## 🚀 Quick Start Commands

```bash
# Build project
pio run

# Upload to ESP32  
pio run --target upload

# Monitor serial output
pio device monitor

# Build and upload in one command
pio run --target upload && pio device monitor
```

## 🔧 Key Components at a Glance

| Component | File | Purpose | CPU Core |
|-----------|------|---------|----------|
| TrainController | `TrainController.cpp` | Main application logic | Core 1 |
| DuploHub | `DuploHub.h/.cpp` | Hardware abstraction | Both |
| BLE Task | Part of DuploHub | BLE communication | Core 0 |
| Lpf2Hub | Legoino library | LEGO protocol | Core 0 |

## 📊 System Status Quick Check

```cpp
// In your main loop, these methods tell you system health:
duploHub.isBLETaskRunning()  // Should return true
duploHub.isConnected()       // true when hub connected
duploHub.isConnecting()      // true during connection attempt
```

## 🎛️ Essential API Calls

```cpp
// Setup (call once in setup())
duploHub.startBLETask();
duploHub.setOnConnectedCallback(myCallback);

// Control (call anytime - thread safe)
duploHub.setMotorSpeed(50);     // 50% forward
duploHub.setMotorSpeed(-50);    // 50% backward  
duploHub.stopMotor();           // stop immediately
duploHub.setLedColor(RED);      // change LED color

// Monitoring (call in main loop)
duploHub.update();              // handle callbacks
```

## 🐛 Quick Troubleshooting

| Problem | Quick Fix |
|---------|-----------|
| Won't connect | Power cycle DUPLO hub, check distance |
| Commands delayed | Check BLE connection, reduce command rate |
| Task won't start | Check free heap memory |
| Connection drops | Move closer, check interference |

## 📈 Performance Tips

- BLE Task runs every 50ms for commands, 1s for connection checks
- Main loop should call `duploHub.update()` frequently (every loop)
- Status logging every 10s is optimal for debugging
- Keep command rate reasonable (max ~10 commands/second)

## 🔍 Debug Output Prefixes

- `TrainController:` - Main app
- `BLE Task:` - Background BLE  
- `DuploHub:` - Hardware layer
- `ERROR:` / `WARNING:` - Issues

## 📝 Common Code Patterns

### Connection Event Handling
```cpp
void onHubConnected() {
    // Setup code here
    duploHub.setHubName("MyTrain");
    startMyApplication();
}

void onHubDisconnected() {
    // Cleanup code here  
    stopMyApplication();
}
```

### Safe Command Sending
```cpp
if (duploHub.isConnected()) {
    duploHub.setMotorSpeed(speed);
} else {
    Serial.println("Hub not connected - command skipped");
}
```

### Non-Blocking Timing
```cpp
static unsigned long lastAction = 0;
if (millis() - lastAction > 1000) {  // Every 1 second
    duploHub.setLedColor(nextColor);
    lastAction = millis();
}
```
