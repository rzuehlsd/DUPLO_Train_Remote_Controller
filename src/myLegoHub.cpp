#include "myLegoHub.h"

/*
 * myLegoHub.cpp
 *
 * Description:
 *   Implementation of LEGO Powered Up/Boost/Control+ Hub abstraction for ESP32.
 *   Provides BLE device discovery, connection, and basic device control. Used as a base for DuploHub.
 *
 * Author: Ralf Zühlsdorff
 * Copyright (c) 2025 Ralf Zühlsdorff
 * License: MIT License
 *
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
#include "myLegoHub.h"


/**
 * @brief Activate the base speaker port device.
 *
 * Activates the speaker port and sets the sound mode for the LEGO hub.
 */
void myLegoHub::activateBaseSpeaker()
{
    // Activate the speaker port device first
    activatePortDevice((byte)DuploTrainHubPort::SPEAKER);
    delay(100); // Give time for activation
    // Set sound mode 
    byte setSoundMode[8] = { 0x41, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01 };
    WriteValue(setSoundMode, 8);
}


/**
 * @brief Play a sound by sound ID.
 * @param sound The sound ID to play.
 *
 * Sends a command to play the specified sound on the hub speaker.
 */
void myLegoHub::playSound(byte sound)
{
    // Then play the sound
    byte playSound[6] = { 0x81, 0x01, 0x11, 0x51, 0x01, sound };
    WriteValue(playSound, 6);
}


/**
 * @brief Activate the RGB LED port device.
 *
 * Activates the RGB LED port for color control.
 */
void myLegoHub::activateRgbLight()
{
    byte port = getPortForDeviceType((byte)DeviceType::HUB_LED);
    byte setColorMode[8] = { 0x41, port, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 };
    WriteValue(setColorMode, 8);
}


/**
 * @brief Set the LED color.
 * @param color The color to set (enum Color).
 *
 * Sets the RGB LED to the specified color.
 */
void myLegoHub::setLedColor(Color color)
{
    byte port = getPortForDeviceType((byte)DeviceType::HUB_LED);
    byte setColor[6] = { 0x81, port, 0x11, 0x51, 0x00, color };
    Lpf2Hub::WriteValue(setColor, 6);
}




/**
 * @brief Map raw speedometer values to normalized -100..100 range.
 * @param rawSpeed Raw speedometer value from sensor (-32768..32767).
 * @param maxRawSpeed Maximum raw speed value for mapping (default 255).
 * @return Normalized speed value (-100..100).
 *
 * Maps the raw speedometer sensor value to a normalized range for application logic.
 */
int myLegoHub::MapSpeedometer(int rawSpeed, int maxRawSpeed)
{
    // Define thresholds for practical speed ranges
    const int MIN_PRACTICAL_SPEED = -maxRawSpeed;
    const int DEAD_ZONE = 10;  // Small values considered as stopped
    // Handle dead zone (near zero speeds)
    if (abs(rawSpeed) <= DEAD_ZONE) {
        return 0;
    }
    // Clamp to practical range to avoid extreme values
    int clampedSpeed = constrain(rawSpeed, MIN_PRACTICAL_SPEED, maxRawSpeed);
    // Map to -100..100 range
    if (clampedSpeed > 0) {
        return map(clampedSpeed, DEAD_ZONE, maxRawSpeed, 1, 100);
    } else {
        return map(clampedSpeed, MIN_PRACTICAL_SPEED, -DEAD_ZONE, -100, -1);
    }
}


/**
 * @brief Parse voltage from sensor data.
 * @param pData Pointer to raw sensor data.
 * @return Voltage as float.
 *
 * Uses Legoino's Lpf2Hub::parseVoltageSensor to extract voltage value.
 */
float myLegoHub::parseVoltage(uint8_t *pData)
{
    // Use Legoino's Lpf2Hub::parseVoltageSensor
    return (float)Lpf2Hub::parseVoltageSensor(pData);
}
