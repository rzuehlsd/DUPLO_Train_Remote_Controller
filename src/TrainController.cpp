/**
 * @file TrainController.cpp
 * @brief Main application for LEGO DUPLO Train control using DuploHub on ESP32.
 *
 * This file implements the main application logic for controlling a LEGO DUPLO train
 * using the DuploHub class and ESP32 dual-core architecture. It demonstrates a modular,
 * event-driven approach to BLE-based train control, with robust handling of motor, LED,
 * and sound commands, as well as real-time sensor feedback and user input.
 *
 * Architecture:
 * - TrainController: Main application logic (this file)
 * - DuploHub: Hardware abstraction layer with multi-task BLE management
 * - Lpf2Hub: Low-level LEGO Powered Up protocol implementation
 *
 * Features:
 * - Dual-core, multi-task architecture (BLE on Core 0, main loop on Core 1)
 * - Thread-safe command queuing for motor, LED, and sound control
 * - Real-time status monitoring, logging, and error handling
 * - Automatic BLE connection management and recovery
 * - Modular callback system for sensor and event handling
 * - Sound and LED demo sequences
 * - Emergency stop and user input via debounced buttons and potentiometer
 *
 * Demo Sequence:
 * - LED color changes (GREEN, RED, etc.)
 * - Motor control (forward, backward, stop)
 * - Sound playback (HORN, BELL, WATER_REFILL, etc.)
 *
 * @author Ralf Zühlsdorff
 * @date 2025
 *
 * @copyright
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "DuploHub.h"
#include "SystemMemory.h"
#include <Bounce2.h>

#define DEBUG 1 // Enable debug logging
#include "debug.h"

#define DELAY_TIME 100 // delay in ms

// Pin declaration
#define BTN_WASSER 25 // Button 3 (blau)
#define BTN_LICHT 27  // Button 2 (weiß)
#define BTN_SOUND 26  // Button 1 (gelb)
#define BTN_STOP 14   // Button 4 (rot)
#define POTI_SPEED 15 // Throttle potentiometer (analog input ADC13)

// TrainController instance with DuploHub for BLE communication
DuploHub duploHub;

// Create instances of the ezButton class
// Debouncing and handling of the buttons
Bounce2::Button pbSound = Bounce2::Button();
Bounce2::Button pbLight = Bounce2::Button();
Bounce2::Button pbWater = Bounce2::Button();
Bounce2::Button pbStop = Bounce2::Button();

static int detectedSpeed = 0;
static float detectedVoltage = 0.0f;
static int detectedColor = 0;
static int potiSpeed = 0;
static bool emergencyStop = false;

/**
 * @brief Handles button presses for sound, light, water, and emergency stop.
 *
 * Cycles through sounds and LED colors, triggers water sound, and toggles emergency stop.
 */
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

/**
 * @brief Updates the state of all debounced buttons.
 */
void updateButtons()
{
    pbSound.update();
    pbLight.update();
    pbWater.update();
    pbStop.update();
}

static int POTI_MIN = 3584; // Minimum value for the potentiometer
static int POTI_MAX = 512;

/**
 * @brief Reads and averages the potentiometer value, updates min/max calibration.
 */
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

// Sets the speed of the currently selected train
#define STEPSIZE 10
#define MAX_SPEED 100
#define NO_STEPS 10
#define MIN_SPEED 20
#define POTI_DELAY 500

/**
 * @brief Maps the potentiometer value to a train speed in the range -100 to 100.
 * @param potiValue The raw potentiometer value to map.
 * @return The mapped speed value.
 */
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

/**
 * @brief Handles potentiometer input and sets train speed accordingly.
 *
 * Stops the train if emergency stop is active or if the potentiometer is in the zero zone.
 */
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
/**
 * @brief Callback for detected color events from the color sensor.
 * @param color The detected color (DuploEnums::DuploColor).
 */
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
    /**
     * @brief Callback for detected voltage events from the voltage sensor.
     * @param voltage The detected voltage value.
     */
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
    /**
     * @brief Callback for detected speed events from the speed sensor.
     * @param speed The detected speed value.
     */
    static void detectedSpeedCb(int speed)
    {
        DEBUG_LOG("TrainController:  Detected Speed: %d", speed);
        detectedSpeed = speed;
    }

    // Callback function when hub connects
    /**
     * @brief Callback function called when the hub is connected.
     *
     * Activates all relevant ports and registers sensor callbacks.
     */
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
    /**
     * @brief Callback function called when the hub is disconnected.
     */
    static void onHubDisconnected()
    {
        DEBUG_LOG("TrainController: Hub disconnected - stopping all operations");
    }

    /**
     * @brief Initializes and configures all button instances for input and debouncing.
     */
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

    /**
     * @brief Arduino setup function. Initializes serial, memory info, buttons, DuploHub, and callbacks.
     */
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

    /**
     * @brief Periodically logs the status of the BLE task, hub connection, and CPU temperature.
     * @param duploHub Reference to the DuploHub instance.
     */
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
    /**
     * @brief Arduino main loop. Handles input, updates train state, and processes hub responses.
     */
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

        delay(10); // Add a small delay to reduce CPU load and prevent guru meditation error
    } // End of loop
