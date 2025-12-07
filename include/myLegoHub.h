/**
 * @file myLegoHub.h
 * @brief Thin wrapper around `Lpf2Hub` adding DUPLO-specific helpers.
 *
 * Declares `myLegoHub`, a small extension of Legoino's base hub with convenience helpers for
 * activating DUPLO ports and parsing sensor data. `DuploHub` builds on this abstraction to offer
 * higher-level features.
 *
 * @author Ralf Zühlsdorff
 * @date 2025
 *
 * @copyright
 * MIT License
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
    /** @brief Enable the DUPLO speaker port. */
    void activateBaseSpeaker();

    /** @brief Start playback of a DUPLO sound effect. */
    void playSound(byte sound);

    /** @brief Enable the on-hub RGB LED port. */
    void activateRgbLight();

    /** @brief Apply a color to the on-hub RGB LED. */
    void setLedColor(Color color);

    /** @brief Map the raw speedometer count into a normalized range. */
    int MapSpeedometer(int rawSpeed, int maxRawSpeed = 255);

    /** @brief Convert raw voltage sensor bytes into volts. */
    float parseVoltage(uint8_t *pData);
};

#endif // MY_LEGO_HUB_H
