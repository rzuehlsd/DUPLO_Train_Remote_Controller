/**
 * TrainController - A complete train control application using DuploHub class
 * Controls a DUPLO train with a motor connected to Port A of the Hub
 * 
 * Architecture:
 * - TrainController: Main application logic (this file)
 * - DuploHub: Hardware abstraction layer with multi-task BLE management
 * - Lpf2Hub: Low-level LEGO Powered Up protocol implementation
 * 
 * Features:
 * - Multi-task architecture with BLE operations in background task (Core 0)
 * - Non-blocking demo sequence running in main loop (Core 1)
 * - Automatic connection management and recovery
 * - Real-time status monitoring and logging
 * - Thread-safe command queuing for motor, LED, and sound control
 * - Sound tests integrated into the demo sequence
 * - Optimized BLE task responsiveness and reduced latency
 * - Improved sound playback timing
 * 
 * Demo Sequence:
 * - LED color changes (GREEN, RED)
 * - Motor control (forward, backward, stop)
 * - Sound playback (HORN, BELL)
 * 
 * (c) Copyright 2025
 * Released under MIT License
 */

#include "DuploHub.h"
#include "SystemMemory.h"    
#include <Bounce2.h>  


#undef DEBUG  // Enable debug logging
#include "debug.h"



#define DELAY_TIME  100     // delay in ms


//Pin declaration
#define BTN_WASSER 25       // Taster 3 (blau)           
#define BTN_LICHT 27        // Taster 2 (weiß)
#define BTN_SOUND 26        // Taster 1 (gelb)
#define BTN_STOP 14         // Taster 4 (rot)
#define POTI_SPEED 15       // Poti Fahrtregler (Analogeingang ADC13)



// TrainController instance with DuploHub for BLE communication
DuploHub duploHub;

// Instanz der ezButton Klasse erstellen
// Debouncing und Handling der Buttons
Bounce2::Button pbSound = Bounce2::Button();
Bounce2::Button pbLight = Bounce2::Button();
Bounce2::Button pbWater = Bounce2::Button();
Bounce2::Button pbStop = Bounce2::Button();



// Buttons werden in der MainLoop aktualisiert
void handleButtons()
{
    static int color = 0; // Color index for LED cycling
    static int duploSounds[] = {
        DuploEnums::DuploSound::HORN,
        DuploEnums::DuploSound::BRAKE,
        DuploEnums::DuploSound::WATER_REFILL,
        DuploEnums::DuploSound::STATION_DEPARTURE,
        DuploEnums::DuploSound::STEAM
    };
    static int sound = 0; // Sound index for sound playback
    

    DEBUG_LOG("TrainController: Handling buttons...");
 
    if(pbSound.pressed())   // Cycle through Duplo sounds
    {
        sound = (sound + 1) % 5;
        duploHub.playSound((DuploEnums::DuploSound)duploSounds[sound]);
        delay(DELAY_TIME);
    }
  
    if(pbLight.pressed())   // cycle through LED colors
    {
        color = (color + 1) % 11; // Cycle through DuploColor enum values
        duploHub.setLedColor((DuploEnums::DuploColor)color);
        delay(DELAY_TIME);
    }
  
    if(pbWater.pressed())
    {
     duploHub.playSound(DuploEnums::DuploSound::WATER_REFILL);
      delay(DELAY_TIME);
    }
  
    if(pbStop.pressed())    // emergency stop
    {
      duploHub.setMotorSpeed(0);
      delay(DELAY_TIME);
      duploHub.playSound((DuploEnums::DuploSound::BRAKE));
      delay(DELAY_TIME);
    }

}


void updateButtons() {
    pbSound.update();
    pbLight.update();
    pbWater.update();
    pbStop.update();
}



// Function to process the responseQueue and print detected color
static void detectedColorCb(DuploEnums::DuploColor color) {
    DEBUG_LOG("TrainController:  Detected Color: %d", color);
}


// Function to process the responseQueue and print detected voltage
static void detectedVoltageCb(float voltage) {
    DEBUG_LOG("TrainController:  Detected Voltage: %.2f V", voltage);
}



// Function to process the responseQueue and print detected color
static void detectedSpeedCb(int speed) {
    DEBUG_LOG("TrainController:  Detected Speed: %d", speed);
}



// Callback function when hub connects
static void onHubConnected() {
    DEBUG_LOG("TrainController: Hub instance activated, starting demo sequence...");

    #ifdef DEBUG
    duploHub.listDevicePorts(); // List connected devices on the hub
    delay(1000);
    #endif
    
    duploHub.activateRgbLight();
    delay(200);
    duploHub.activateBaseSpeaker();
    delay(200);
    duploHub.activateColorSensor();
    delay(200);
    duploHub.activateSpeedSensor();
    delay(200);
    duploHub.activateVoltageSensor();
    delay(200);
    DEBUG_LOG("DuploHub: All ports activated");
    
    duploHub.setDetectedColorCallback(detectedColorCb);
    delay(200);
    duploHub.setDetectedSpeedCallback(detectedSpeedCb);
    delay(200);
    duploHub.setDetectedVoltageCallback(detectedVoltageCb);
    delay(200);
    DEBUG_LOG("DuploHub: All callbacks registered");

    duploHub.setHubName("DuploTrain_1");
    DEBUG_LOG("TrainController: Connected to hub: %s", duploHub.getHubName().c_str());
    DEBUG_LOG("TrainController: Hub address: %s", duploHub.getHubAddress().c_str());
}



// Callback function when hub disconnects
static void onHubDisconnected() {
    DEBUG_LOG("TrainController: Hub disconnected - stopping all operations");

}

void setupButtons() {
    // Initialize ezButton instances
    pbWater.attach(BTN_WASSER, INPUT_PULLUP);
    pbLight.attach(BTN_LICHT, INPUT_PULLUP);
    pbSound.attach(BTN_SOUND, INPUT_PULLUP);
    pbStop.attach(BTN_STOP, INPUT_PULLUP);

    // Set initial debounce time
    pbWater.interval(5); // Debounce time in ms
    pbLight.interval(5);
    pbSound.interval(5);
    pbStop.interval(5);

    pbWater.setPressedState(LOW); // Set pressed state for buttons
    pbLight.setPressedState(LOW);
    pbSound.setPressedState(LOW);
    pbStop.setPressedState(LOW);
}






void setup() {
    Serial.begin(115200);
    delay(5000);

    //  SerialMUTEX();
    DEBUG_LOG("TrainController (2 Core) : Starting up...");
    DEBUG_LOG("");

    printMemoryInfo();

    // Button initialisieren
    setupButtons();

    // Initialize DuploHub instance
    duploHub.init();
    delay(1000);

    // Register connection event callbacks
    duploHub.setOnConnectedCallback(onHubConnected);
    delay(200);
    duploHub.setOnDisconnectedCallback(onHubDisconnected);
    delay(200); // Allow time for BLE task to initialize

    DEBUG_LOG("TrainController: Ready - BLE task running, waiting for hub connection...");
} 





void checkStatus(DuploHub& duploHub) {
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 30000) { // Every 30 seconds
        DEBUG_LOG("TrainController Status - BLE Task: %s, Hub Connected: %s",
                  duploHub.isBLETaskRunning() ? "Running" : "Stopped",
                  duploHub.isConnected() ? "Yes" : "No");
        float cpuTemp = getCPUTemperature();
        DEBUG_LOG("CPU Temperature: %.2f °C", cpuTemp);
        lastStatusUpdate = millis();
    }
}




// main loop
void loop() {
    updateButtons();

    if(duploHub.isConnected()) {
        handleButtons();
        duploHub.processResponseQueue();
    }
    
    // Check Duplo Hub State
    // checkStatus(duploHub);

    delay(10); // Add a small delay to reduce CPU load
} // End of loop

