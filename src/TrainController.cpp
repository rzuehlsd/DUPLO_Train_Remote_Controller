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

#define DEBUG 1 // Enable debug logging
#include "debug.h"

#define DELAY_TIME 100 // delay in ms

// Pin declaration
#define BTN_WASSER 25 // Taster 3 (blau)
#define BTN_LICHT 27  // Taster 2 (weiß)
#define BTN_SOUND 26  // Taster 1 (gelb)
#define BTN_STOP 14   // Taster 4 (rot)
#define POTI_SPEED 15 // Poti Fahrtregler (Analogeingang ADC13)

// TrainController instance with DuploHub for BLE communication
DuploHub duploHub;

// Instanz der ezButton Klasse erstellen
// Debouncing und Handling der Buttons
Bounce2::Button pbSound = Bounce2::Button();
Bounce2::Button pbLight = Bounce2::Button();
Bounce2::Button pbWater = Bounce2::Button();
Bounce2::Button pbStop = Bounce2::Button();

static int detectedSpeed = 0;
static float detectedVoltage = 0.0f;
static int detectedColor = 0;
static int potiSpeed = 0;
static bool emergencyStop = false;

// Buttons werden in der MainLoop aktualisiert
void handleButtons()
{
    static int color = 0; // Color index for LED cycling
    static int duploSounds[] = {
        DuploEnums::DuploSound::HORN,
        DuploEnums::DuploSound::BRAKE,
        DuploEnums::DuploSound::WATER_REFILL,
        DuploEnums::DuploSound::STATION_DEPARTURE,
        DuploEnums::DuploSound::STEAM};
    static int sound = 0; // Sound index for sound playback

    if (pbSound.pressed())
    { // Cycle through Duplo sounds
        sound = (sound + 1) % 5;
        duploHub.playSound((DuploEnums::DuploSound)duploSounds[sound]);
        delay(DELAY_TIME);
    }

    if (pbLight.pressed())
    {                             // cycle through LED colors
        color = (color + 1) % 11; // Cycle through DuploColor enum values
        duploHub.setLedColor((DuploEnums::DuploColor)color);
        delay(DELAY_TIME);
    }

    if (pbWater.pressed())
    {
        duploHub.playSound(DuploEnums::DuploSound::WATER_REFILL);
        delay(DELAY_TIME);
    }

    if (pbStop.pressed())
    {                                   // emergency stop
        emergencyStop = !emergencyStop; // Toggle emergency stop state
        delay(DELAY_TIME);
        if (emergencyStop)
        {
            duploHub.playSound((DuploEnums::DuploSound::BRAKE)); // Play brake sound
            delay(DELAY_TIME);                                   // Allow time for sound to play
            duploHub.setLedColor(DuploEnums::DuploColor::RED);
            delay(DELAY_TIME);
        }
        else
        {
            duploHub.playSound((DuploEnums::DuploSound::STATION_DEPARTURE)); // Play departure sound
            delay(DELAY_TIME);                                               // Allow time for sound to play
            duploHub.setLedColor(DuploEnums::DuploColor::GREEN);             // Set LED to green
            delay(DELAY_TIME);                                               // Allow time for sound to play
        }

        DEBUG_LOG("TrainController: Emergency stop %s", emergencyStop ? "activated" : "deactivated");
        delay(DELAY_TIME);
    }
}

void updateButtons()
{
    pbSound.update();
    pbLight.update();
    pbWater.update();
    pbStop.update();
}

static int POTI_MIN = 3584; // Minimum value for the potentiometer
static int POTI_MAX = 512;

void updatePoti()
{
    // read adc value
    for (int i = 0; i < 5; i++)
    {
        potiSpeed = potiSpeed + analogRead(POTI_SPEED);
    }
    potiSpeed /= 5;

    POTI_MIN = min(POTI_MIN, potiSpeed);
    POTI_MAX = max(POTI_MAX, potiSpeed);
    // DEBUG_LOG("poti = %d. min = %d, max = %d", potiSpeed, POTI_MIN, POTI_MAX);
}

// setzt die Geschwindigkeit des aktuell selektierten Zuges
#define STEPSIZE 10
#define MAX_SPEED 100
#define NO_STEPS 10
#define MIN_SPEED 20
#define POTI_DELAY 500

int mapPoti(int potiValue)
{
    // Map the poti value to a speed range
    int STEP_SIZE = ((POTI_MAX - POTI_MIN) / (NO_STEPS * 2 + 1));
    int steps = (potiValue - POTI_MIN) / STEP_SIZE;
    int speed = (steps - NO_STEPS) * MAX_SPEED / NO_STEPS; // Scale to speed range
    if (speed < -100)
        speed = -100; // Limit minimum speed
    if (speed > 100)
        speed = 100; // Limit maximum speed
    return speed;
}

static int lastSpeed = 0; // Store last poti speed for comparison

void handlePoti()
{
    int lastSpeed = 0; // Store last poti speed for comparison
    int speed = 0;     // holds current mapping of poti setting to speed
    static long lastCall = millis();

    if (emergencyStop)
    {
        duploHub.setMotorSpeed(0); // Stop the motor immediately
        lastSpeed = 0;             // Reset lastSpeed to avoid repeated calls
        delay(DELAY_TIME);
        DEBUG_LOG("TrainController: Emergency stop activated, motor speed set to 0");
        return; // Exit if emergency stop is active
    }

    if (millis() - lastCall > POTI_DELAY)
    {
        lastCall = millis(); // Update last call time

        speed = mapPoti(potiSpeed); // map current poti setting to speed

        // if emergencyStop triggered or poti in zero zone
        if (speed >= -MIN_SPEED && speed <= MIN_SPEED)
        {
            duploHub.setMotorSpeed(0);
            lastSpeed = 0; // Reset lastSpeed to avoid repeated calls
            delay(DELAY_TIME);
            DEBUG_LOG("TrainController: Set motor speed to %d", lastSpeed);
        }

        if (abs(lastSpeed - speed) > STEPSIZE) // if speed changed
        {
            // train speed changed
            duploHub.setMotorSpeed(speed);
            lastSpeed = speed;
            delay(DELAY_TIME);
            DEBUG_LOG("TrainController: Set motor speed to %d", lastSpeed);
        }
    }
}

// Function to process the responseQueue and print detected color
static void detectedColorCb(DuploEnums::DuploColor color)
{
    DEBUG_LOG("TrainController:  Detected Color: %d", color);
    detectedColor = color; // Store the detected color
    switch (color)
    {
    case DuploEnums::DuploColor::BLACK:
        duploHub.setLedColor(DuploEnums::DuploColor::BLACK);
        delay(DELAY_TIME);
        break;
    case DuploEnums::DuploColor::RED:
        emergencyStop = !emergencyStop; // Toggle emergency stop state
        delay(DELAY_TIME);
        duploHub.playSound((DuploEnums::DuploSound::BRAKE)); // Play brake sound
        delay(DELAY_TIME);                                   // Allow time for sound to play
        duploHub.setLedColor(DuploEnums::DuploColor::RED);
        delay(DELAY_TIME);
        break;
    case DuploEnums::DuploColor::YELLOW:
        duploHub.setLedColor(DuploEnums::DuploColor::YELLOW);
        delay(DELAY_TIME);
        duploHub.playSound((DuploEnums::DuploSound::HORN)); // Play horn sound
        delay(DELAY_TIME);                                  // Allow time for sound to play
        break;
    case DuploEnums::DuploColor::BLUE:
        duploHub.setLedColor(DuploEnums::DuploColor::BLUE);
        delay(DELAY_TIME);
        duploHub.stopMotor();
        delay(DELAY_TIME);                                       // Stop the motor
        duploHub.playSound((DuploEnums::DuploSound::WATER_REFILL)); // Play water refill sound
        delay(DELAY_TIME);                                          // Allow time for sound to play
        duploHub.setMotorSpeed(lastSpeed);                          // Restore the last speed
        delay(DELAY_TIME);
        break;
    case DuploEnums::DuploColor::WHITE:
        for (int i = 0; i < 3; i++)
        {
            duploHub.setLedColor(DuploEnums::DuploColor::WHITE);
            delay(DELAY_TIME);
            duploHub.setLedColor(DuploEnums::DuploColor::BLACK);
            delay(DELAY_TIME);
        }
        break;
    }
}

    // Function to process the responseQueue and print detected voltage
    static void detectedVoltageCb(float voltage)
    {
        static float meanVoltage[10] = {0.0f};
        static int i = 0;
        DEBUG_LOG("TrainController:  Detected Voltage: %.2f V", voltage);
        i = (i + 1) % 10; // Circular buffer
        float sum = 0.0f;
        for (int j = 0; j < 10; j++)
        {
            sum += meanVoltage[j];
        }
        if (i == 10)
            detectedVoltage = sum / 10.0f; // Calculate mean voltage
    }

    // Function to process the responseQueue and print detected color
    static void detectedSpeedCb(int speed)
    {
        DEBUG_LOG("TrainController:  Detected Speed: %d", speed);
        detectedSpeed = speed;
    }

    // Callback function when hub connects
    static void onHubConnected()
    {
        DEBUG_LOG("TrainController: Hub instance activated, starting demo sequence...");

#ifdef DEBUG
        duploHub.listDevicePorts(); // List connected devices on the hub
        delay(1000);
#endif

        duploHub.activateRgbLight();
        delay(500);
        duploHub.activateBaseSpeaker();
        delay(500);
        duploHub.activateColorSensor();
        delay(500);
        // duploHub.activateSpeedSensor();
        delay(200);
        // duploHub.activateVoltageSensor();
        delay(200);
        DEBUG_LOG("DuploHub: All ports activated");

        duploHub.setDetectedColorCallback(detectedColorCb);
        delay(200);
        // duploHub.setDetectedSpeedCallback(detectedSpeedCb);
        delay(200);
        // duploHub.setDetectedVoltageCallback(detectedVoltageCb);
        delay(200);
        DEBUG_LOG("DuploHub: All callbacks registered");

        duploHub.setHubName("DuploTrain_1");
        DEBUG_LOG("TrainController: Connected to hub: %s", duploHub.getHubName().c_str());
        DEBUG_LOG("TrainController: Hub address: %s", duploHub.getHubAddress().c_str());
    }

    // Callback function when hub disconnects
    static void onHubDisconnected()
    {
        DEBUG_LOG("TrainController: Hub disconnected - stopping all operations");
    }

    void setupButtons()
    {
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

    void setup()
    {
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

    void checkStatus(DuploHub & duploHub)
    {
        static unsigned long lastStatusUpdate = 0;
        if (millis() - lastStatusUpdate > 30000)
        { // Every 30 seconds
            DEBUG_LOG("TrainController Status - BLE Task: %s, Hub Connected: %s",
                      duploHub.isBLETaskRunning() ? "Running" : "Stopped",
                      duploHub.isConnected() ? "Yes" : "No");
            float cpuTemp = getCPUTemperature();
            DEBUG_LOG("CPU Temperature: %.2f °C", cpuTemp);
            lastStatusUpdate = millis();
        }
    }

    // main loop
    void loop()
    {
        updateButtons();
        updatePoti();

        if (duploHub.isConnected())
        {
            handleButtons();
            handlePoti();
            duploHub.processResponseQueue();
        }

        // Check Duplo Hub State
        checkStatus(duploHub);

        delay(10); // Add a small delay to reduce CPU load
    } // End of loop
