/*
 * myLegoHub.h
 *
 * Description:
 *   C++ abstraction for LEGO Powered Up/Boost/Control+ Hubs on ESP32.
 *   Provides BLE communication, device discovery, and basic device control. Used as a base for DuploHub.
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
#ifndef MY_LEGO_HUB_H
#define MY_LEGO_HUB_H

#include "Lpf2Hub.h"
#include "LegoinoCommon.h"

namespace DuploEnums
{
    // Device types for Duplo Train Hub
    enum struct DuploTrainDeviceType
    {
        DUPLO_TRAIN_BASE_MOTOR = 41,
        DUPLO_TRAIN_BASE_SPEAKER = 42,
        DUPLO_TRAIN_BASE_COLOR_SENSOR = 43,
        DUPLO_TRAIN_BASE_SPEEDOMETER = 44,
        VOLTAGE_SENSOR = 20,
        LIGHT = 8
    };

    // Port enumeration for Duplo Train Hub
    // These ports correspond to the physical ports on the DUPLO Train Hub
    // and are used for device activation and communication.
    enum struct DuploTrainHubPort
    {
        MOTOR = 0x00,
        LED = 0x11,
        SPEAKER = 0x01,
        COLOR = 0x12,
        SPEEDOMETER = 0x13,
        VOLTAGE = 0x14
    };

    

    // Enum for available Duplo sounds
    enum DuploSound
    {
        BRAKE = 3,
        STATION_DEPARTURE = 5,
        WATER_REFILL = 7,
        HORN = 9,
        STEAM = 10
    };

    // Enum for available Duplo colors
    enum DuploColor
    {
        BLACK = 0,
        PINK = 1,
        PURPLE = 2,
        BLUE = 3,
        LIGHT_BLUE = 4,
        CYAN = 5,
        GREEN = 6,
        YELLOW = 7,
        ORANGE = 8,
        RED = 9,
        WHITE = 10
    };

    
}

// Callback function types
typedef void (*ConnectionCallback)();

// Add a typedef for the detected color callback
typedef void (*DetectedColorCallback)(DuploEnums::DuploColor);

// Add a typedef for the speed callback
typedef void (*DetectedSpeedCallback)(int detectedSpeed);

// Add a typedef for the voltage callback
typedef void (*DetectedVoltageCallback)(float detectedVoltage);




// Class myLegoHub adds extensions and small modifucations to the Lpf2Hub class

class myLegoHub : public Lpf2Hub {
public:
    /**
     * @brief Activate the base speaker port device.
     */
    void activateBaseSpeaker();

    /**
     * @brief Play a sound by sound ID.
     * @param sound The sound ID to play.
     */
    void playSound(byte sound);

    /**
     * @brief Activate the RGB LED port device.
     */
    void activateRgbLight();

    /**
     * @brief Set the LED color.
     * @param color The color to set (enum Color).
     */
    void setLedColor(Color color);

    /**
     * @brief Map raw speedometer value to -100..100 range.
     * @param rawSpeed Raw speedometer value from sensor.
     * @param maxRawSpeed Maximum raw speed value for mapping (default 255).
     * @return Normalized speed value (-100..100).
     */
    int MapSpeedometer(int rawSpeed, int maxRawSpeed = 255);

    /**
     * @brief Parse voltage from sensor data.
     * @param pData Pointer to raw sensor data.
     * @return Voltage as float.
     */
    float parseVoltage(uint8_t *pData);
};

#endif // MY_LEGO_HUB_H
