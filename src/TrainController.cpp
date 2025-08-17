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

/*
NEXT GENERATION DUPLO TRAIN CONTROLLER

ONLY ON ESP§"S§

*/

#include "DuploHub.h"
#include "SystemMemory.h"

#undef DEBUG 1 // Enable debug logging
#include "debug.h"
#include "ADCButton.h"
// on board RGB LED
#include "StatusLED.h"

#include <ESP32RotaryEncoder.h>

#define DELAY_TIME 100 // delay in ms

// Pin declaration
// All Buttons are connected via resistor array to ADC pin 7
// Button setup uses voltage divider with different resistor values for each button
// ADC thresholds need to be calibrated based on actual resistor values:
// - Sound button: ~3000-3500 ADC value
// - Light button: ~2500-3000 ADC value
// - Water button: ~2000-2500 ADC value
// - Stop button:  ~1500-2000 ADC value
// Use calibrateButtons() function to determine actual values for your hardware
#define ADC_PIN 7 // Button ADC pin (GPIO 7)

// Speed is defined via rotary encoder
#define ENCODER_A 4 // Encoder A pin
#define ENCODER_B 5 // Encoder B pin

// Create rotary encoder instance
RotaryEncoder rotaryEncoder = RotaryEncoder(ENCODER_A, ENCODER_B); // -1 means no button pin

// Onboard RGB LED
#define DATA_PIN 48

// define low voltage threshold for voltage sensor
#define LOW_VOLTAGE_THRESHOLD 6.0f // Voltage threshold for low battery warning

StatusLED statusLed(DATA_PIN); // Pin 48 for ESP32-S3-DevKitC-1

// TrainController instance with DuploHub for BLE communication
DuploHub duploHub;


// Determine which button is pressed based on ADC value
static bool soundPressed = false;
static bool lightPressed = false;
static bool recordPressed = false;
static bool playPressed = false;
static bool stopPressed = false;

static int detectedSpeed = 0;
static float detectedVoltage = 0.0f;
static int detectedColor = 0;
static int potiSpeed = 0;
static bool emergencyStop = false;

static bool recording = false;
static bool playback = false;
static bool replay = false; // true when a sequence is being replayed

ADCButtons guiButtons = ADCButtons(ADC_PIN);

// Button numbers for GUI buttons
enum GUIButtons
{
    BUTTON_RECORD,
    BUTTON_SOUND,
    BUTTON_PLAY,
    BUTTON_LIGHT,
    BUTTON_STOP
};

// mapping between Duplo Colors and CRGB Colors
// index corresponds to DuploColor enum values
// 0 = BLACK, 1 = PINK, 2 = PURPLE, etc.
CRGB::HTMLColorCode rgbColors[] = {
    CRGB::Black,
    CRGB::Pink,
    CRGB::Purple,
    CRGB::Blue,
    CRGB::LightBlue,
    CRGB::Cyan,
    CRGB::Green,
    CRGB::Yellow,
    CRGB::Orange,
    CRGB::Red,
    CRGB::White};

/**
 * @brief Handles button presses for sound, light, water, and emergency stop.
 *
 * Cycles through sounds and LED colors, triggers water sound, and toggles emergency stop.
 */
void handleButtons(int btn_no, bool pressed)
{

    static int color = 0; // Color index for LED cycling
    static int duploSounds[] = {
        DuploEnums::DuploSound::HORN,
        DuploEnums::DuploSound::BRAKE,
        DuploEnums::DuploSound::WATER_REFILL,
        DuploEnums::DuploSound::STATION_DEPARTURE,
        DuploEnums::DuploSound::STEAM};
    static int sound = 0; // Sound index for sound playbook

    if (pressed == false || !duploHub.isConnected())
        return;

    // If replay is active, only STOP and PLAY (replay) buttons are allowed
    if (replay && btn_no != BUTTON_STOP && btn_no != BUTTON_PLAY) {
        DEBUG_LOG("TrainController: Ignoring button %d during replay", btn_no);
        return;
    }

    switch (btn_no)
    {
    case BUTTON_SOUND:
        sound = (sound + 1) % 5;
        duploHub.playSound((DuploEnums::DuploSound)duploSounds[sound]);
        DEBUG_LOG("TrainController: Sound button pressed, playing sound %d", sound);
        delay(DELAY_TIME);
        break;
    case BUTTON_LIGHT:
        color = (color + 1) % 11; // Cycle through DuploColor enum values
        statusLed.setColor(rgbColors[color]);
        statusLed.setBlinking(false);
        duploHub.setLedColor((DuploEnums::DuploColor)color);
        DEBUG_LOG("TrainController: Light button pressed, color %d", color);
        delay(DELAY_TIME);
        break;
    case BUTTON_RECORD:
        DEBUG_LOG("TrainController: start or stop recording");
        recording = !recording; // Toggle recording state
        // stop playback if recording is active
        if (recording)
        {
            DEBUG_LOG("TrainController: Recording started");
            statusLed.setColor(CRGB::Blue);           // Blink blue when recording
            statusLed.setBlinking(true, 250, 250);
            duploHub.recordCommands(true); // Start recording commands
            delay(DELAY_TIME);             // Allow time for sound to play
        }
        else
        {
            DEBUG_LOG("TrainController: Recording stopped");
            duploHub.recordCommands(false); // Stop recording commands
            statusLed.setOff();             // LED off when recording stops
            delay(DELAY_TIME); // Allow time for sound to play
        }
        break;
    case BUTTON_PLAY:
        DEBUG_LOG("TrainController: start or stop playback");
        // disable playback when recording is active
        if (recording)
        {
            DEBUG_LOG("TrainController: Playback disabled while recording is active");
            return; // Ignore playback if recording is active
        }
        playback = !playback; // Toggle playback state
        if (playback)
        {
            duploHub.replayCommands();
            statusLed.setColor(CRGB::Yellow);           // Blink yellow during replay
            statusLed.setBlinking(true, 250, 250);
            replay = true; // Set replay mode
        }
        else
        {
            duploHub.stopReplay();
            statusLed.setOff();
            replay = false; // End replay mode
        }
        DEBUG_LOG("TrainController: Playback %s", playback ? "started" : "stopped");
        delay(DELAY_TIME);
        break;
    case BUTTON_STOP:
        emergencyStop = !emergencyStop; // Toggle emergency stop state
        // If replay is active, stop replay and playback immediately
        if (replay) {
            duploHub.stopReplay();
            statusLed.setOff();
            playback = false;
            replay = false;
            DEBUG_LOG("TrainController: STOP pressed during replay, replay stopped");
            return;
        }
        if (emergencyStop)
        {
            statusLed.setColor(CRGB::Red); // Solid red when stop is active
            statusLed.setBlinking(false);
            duploHub.playSound((DuploEnums::DuploSound::BRAKE)); // Play brake sound
            delay(DELAY_TIME);                                   // Allow time for sound to play
            duploHub.setLedColor(DuploEnums::DuploColor::RED);
            delay(DELAY_TIME);
            rotaryEncoder.setEncoderValue(0);
        }
        else
        {
            duploHub.playSound((DuploEnums::DuploSound::STATION_DEPARTURE)); // Play departure sound
            delay(DELAY_TIME);                                               // Allow time for sound to play
            duploHub.setLedColor(DuploEnums::DuploColor::GREEN);             // Set LED to green
            statusLed.setOff(); // LED off when stop is released
            delay(DELAY_TIME); // Allow time for sound to play
        }
        DEBUG_LOG("TrainController: Emergency stop %s", emergencyStop ? "activated" : "deactivated");
        delay(DELAY_TIME);
        break;
    default:
        DEBUG_LOG("TrainController: Unknown button %d", btn_no);
        return; // Ignore unknown buttons
    }
}

// Sets the speed of the currently selected train
#define MIN_SPEED 20
#define POTI_DELAY 250

static int lastSpeed = 0; // holds current mapping of poti setting to speed

/**
 * @brief Handles potentiometer input and sets train speed accordingly.
 *
 * Stops the train if emergency stop is active or if the potentiometer is in the zero zone.
 */
void handleEncoder(long speed)
{
    static long lastCall = millis();

    DEBUG_LOG("TrainController: Set motor speed to %d", speed);

    if (emergencyStop)
    {
        duploHub.setMotorSpeed(0);        // Stop the motor immediately
        rotaryEncoder.setEncoderValue(0); // Reset encoder value
        delay(DELAY_TIME);
        DEBUG_LOG("TrainController: Emergency stop activated, motor speed set to 0");
        return; // Exit if emergency stop is active
    }

    if (millis() - lastCall > POTI_DELAY)
    {
        lastCall = millis(); // Update last call time

        // if emergencyStop triggered or poti in zero zone
        if (speed > -MIN_SPEED && speed < MIN_SPEED)
        {
            duploHub.setMotorSpeed(0);
            lastSpeed = 0; // Reset lastSpeed to avoid repeated calls
        }
        else
        {
            // train speed changed
            duploHub.setMotorSpeed(speed);
            lastSpeed = speed;
        }
        delay(DELAY_TIME);
        DEBUG_LOG("TrainController: Set motor speed to %d", lastSpeed);
    }
}


// Function to process the responseQueue and print detected color
/**
 * @brief Callback for detected color events from the color sensor.
 * @param color The detected color (DuploEnums::DuploColor).
 */
static void detectedColorCb(DuploEnums::DuploColor color)
{
    if (replay) {
        DEBUG_LOG("TrainController: Color sensor callback ignored during replay");
        return;
    }
    DEBUG_LOG("TrainController:  Detected Color: %d", color);
    detectedColor = color; // Store the detected color
    switch (color)
    {
    case DuploEnums::DuploColor::BLACK:
        duploHub.setLedColor(DuploEnums::DuploColor::BLACK);
        statusLed.setColor(CRGB::Black);
        delay(DELAY_TIME);
        break;
    case DuploEnums::DuploColor::RED:
        emergencyStop = !emergencyStop; // Toggle emergency stop state
        statusLed.setColor(CRGB::Red);
        statusLed.setBlinking(true, 250, 250, 3); // Set LED color to white
        delay(DELAY_TIME);
        duploHub.playSound((DuploEnums::DuploSound::BRAKE)); // Play brake sound
        delay(DELAY_TIME);                                   // Allow time for sound to play
        duploHub.setLedColor(DuploEnums::DuploColor::RED);
        delay(DELAY_TIME);
        break;
    case DuploEnums::DuploColor::YELLOW:
        duploHub.setLedColor(DuploEnums::DuploColor::YELLOW);
        statusLed.setColor(CRGB::Yellow);
        statusLed.setBlinking(true, 250, 250, 3); // Set LED color to white
        delay(DELAY_TIME);
        duploHub.playSound((DuploEnums::DuploSound::HORN)); // Play horn sound
        delay(DELAY_TIME);                                  // Allow time for sound to play
        break;
    case DuploEnums::DuploColor::BLUE:
        duploHub.setLedColor(DuploEnums::DuploColor::BLUE);
        statusLed.setColor(CRGB::Blue);
        statusLed.setBlinking(true, 250, 250, 3); // Set LED color to white
        delay(DELAY_TIME);
        duploHub.stopMotor();
        delay(DELAY_TIME);                                          // Stop the motor
        duploHub.playSound((DuploEnums::DuploSound::WATER_REFILL)); // Play water refill sound
        delay(DELAY_TIME);                                          // Allow time for sound to play
        duploHub.setMotorSpeed(lastSpeed);                          // Restore the last speed
        delay(DELAY_TIME);
        break;
    case DuploEnums::DuploColor::WHITE:
        for (int i = 0; i < 3; i++)
        {
            duploHub.setLedColor(DuploEnums::DuploColor::WHITE);
            statusLed.setColor(CRGB::White);
            statusLed.setBlinking(true, 250, 250, 3); // Set LED color to white
        }
        break;
    }
}

// Function to process the responseQueue and print detected voltage
/**
 * @brief Callback for detected voltage events from the voltage sensor.
 * @param voltage The detected voltage value.
 */
// Only execute callback if time since last execution > CALC_VOLTAGE_DELAY
#ifndef CALC_VOLTAGE_DELAY
#define CALC_VOLTAGE_DELAY 5000 // ms, default if not defined elsewhere
#endif
static void detectedVoltageCb(float voltage)
{
    static unsigned long lastExec = 0;
    unsigned long now = millis();
    if (now - lastExec < CALC_VOLTAGE_DELAY) {
        return;
    }
    lastExec = now;
    DEBUG_LOG("TrainController:  Detected Voltage: %.2f V", voltage);
    detectedVoltage = voltage;
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
    DEBUG_LOG("TrainController: Hub instance connected, starting initialization...");
    statusLed.setColor(CRGB::Blue);           // Set color to blue
    statusLed.setBlinking(true, 200, 300, 3); // Blink blue 3x
    delay(1800);                              // Wait for 3 blinks (3 * (200+300) ms)
    statusLed.setOff();                       // Turn LED off after blinking

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
    duploHub.activateVoltageSensor();
    delay(200);
    DEBUG_LOG("DuploHub: All ports activated");

    duploHub.setDetectedColorCallback(detectedColorCb);
    delay(200);
    // duploHub.setDetectedSpeedCallback(detectedSpeedCb);
    delay(200);
    duploHub.setDetectedVoltageCallback(detectedVoltageCb);
    delay(200);
    DEBUG_LOG("DuploHub: All callbacks registered");

    duploHub.setHubName("DuploTrain_1");
    delay(200);
    

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
    statusLed.setColor(CRGB::White);          // Set color to white
    statusLed.setBlinking(true, 200, 300, 3); // Blink white 3x
    delay(1800);                              // Wait for 3 blinks (3 * (200+300) ms)
    statusLed.setOff();                       // Turn LED off after blinking
}

void replayNextCommand()
{
    const HubCommand *cmd = duploHub.getNextReplayCommand();
    if (cmd == nullptr)
    {
        // If replay is no longer active, stop playback and update LED
        if (!duploHub.isReplayActive()) {
            playback = false;
            replay = false;
            statusLed.setBlinking(false); // Stop blinking first
            statusLed.setOff();           // LED off when replay ends
            DEBUG_LOG("TrainController: No more commands to replay, stopping replay");
        } else {
            DEBUG_LOG("TrainController: Not the time to replay command");
        }
        return;
    }
    DEBUG_LOG("TrainController: Replaying command type %d", cmd->type);
    if (!emergencyStop)
    {
        // Execute the command based on its type
        switch (cmd->type)
        {
        case CMD_MOTOR_SPEED:
            duploHub.setMotorSpeed(cmd->data.motor.speed);
            break;
        case CMD_STOP_MOTOR:
            duploHub.stopMotor();
            break;
        case CMD_SET_LED_COLOR:
            duploHub.setLedColor(cmd->data.led.color);
            break;
        case CMD_PLAY_SOUND:
            duploHub.playSound(cmd->data.sound.soundId);
            break;
        case CMD_SET_HUB_NAME:
            duploHub.setHubName(cmd->data.hubName.name);
            break;
        case CMD_ACTIVATE_RGB_LIGHT:
            duploHub.activateRgbLight();
            break;
        case CMD_ACTIVATE_BASE_SPEAKER:
            duploHub.activateBaseSpeaker();
            break;
        case CMD_ACTIVATE_COLOR_SENSOR:
            duploHub.activateColorSensor();
            break;
        case CMD_ACTIVATE_SPEED_SENSOR:
            duploHub.activateSpeedSensor();
            break;
        case CMD_ACTIVATE_VOLTAGE_SENSOR:
            duploHub.activateVoltageSensor();
            break;
        default:
            DEBUG_LOG("TrainController: Unknown command type %d in replay", cmd->type);
            break;
        }
        DEBUG_LOG("TrainController: Replayed command type %d", cmd->type);
    }
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

    // Initialize onboard RGB LED
    statusLed.begin();                     // Initialize FastLED
    statusLed.setBrightness(100);          // Set brightness
    statusLed.setColor(CRGB::Yellow);         // Set color to red
    statusLed.setBlinking(true, 200, 300); // Blink: 200ms on, 300ms off

    // setup push buttons and register callback
    guiButtons.addButton(BUTTON_RECORD, 790);         // Button 1: Record
    guiButtons.addButton(BUTTON_SOUND, 1514);         // Button 2: Sound
    guiButtons.addButton(BUTTON_PLAY, 2313);          // Button 3: Play
    guiButtons.addButton(BUTTON_LIGHT, 3177);         // Button 4: Light
    guiButtons.addButton(BUTTON_STOP, 0);             // Button 5: Stop (emergency stop)
    guiButtons.registerButtonCallback(handleButtons); // Register button callback

    // initialize rotary encoder and register callback
    rotaryEncoder.setEncoderType(EncoderType::FLOATING);
    rotaryEncoder.setBoundaries(-100, 100, false);
    rotaryEncoder.setStepValue(5);
    rotaryEncoder.setEncoderValue(0);
    rotaryEncoder.onTurned(&handleEncoder);
    // This is where the inputs are configured and the interrupts get attached
    rotaryEncoder.begin();

    // Initialize DuploHub instance
    duploHub.init();
    delay(1000);

    // Register connection event callbacks
    duploHub.setOnConnectedCallback(onHubConnected);
    delay(200);
    duploHub.setOnDisconnectedCallback(onHubDisconnected);
    delay(200); // Allow time for BLE task to initialize

    DEBUG_LOG("TrainController: Ready - BLE task running, waiting for hub connection...");
    statusLed.setColor(CRGB::Green);       // Set color to green after initialization
    statusLed.setBlinking(false);          // Solid green
}

/**
 * @brief Periodically logs the status of the BLE task, hub connection, and CPU temperature.
 * @param duploHub Reference to the DuploHub instance.
 */
void checkStatus(DuploHub &duploHub)
{
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 30000)
    { // Every 30 seconds
        DEBUG_LOG("TrainController Status - BLE Task: %s, Hub Connected: %s",
                  duploHub.isBLETaskRunning() ? "Running" : "Stopped",
                  duploHub.isConnected() ? "Yes" : "No");
        float cpuTemp = getCPUTemperature();
        DEBUG_LOG("CPU Temperature: %.2f °C", cpuTemp);

        if(detectedVoltage < LOW_VOLTAGE_THRESHOLD)
        {
            DEBUG_LOG("TrainController: Low voltage detected: %.2f V", detectedVoltage);
            statusLed.setColor(CRGB::Red); // Set LED to red for low voltage
            statusLed.setBlinking(true, 200, 200, 10); // Blink red
        }
    
        lastStatusUpdate = millis();
    }
}

// main loop
/**
 * @brief Arduino main loop. Handles input, updates train state, and processes hub responses.
 */
void loop()
{
    statusLed.update(); // Handle blinking
    guiButtons.updateButtons();

    if (duploHub.isConnected())
    {
        // Handle replay commands if replay is active
        if (duploHub.isReplayActive())
        {
            replayNextCommand();
        }

         duploHub.processResponseQueue();
    }

    // Check Duplo Hub State
    checkStatus(duploHub);

    delay(20); // Add a small delay to reduce CPU load and prevent guru meditation error
} // End of loop
