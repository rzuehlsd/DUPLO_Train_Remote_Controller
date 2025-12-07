/**
 * @file myLegoHub.cpp
 * @brief DUPLO-specific extensions for Legoino's `Lpf2Hub` implementation.
 *
 * Implements lightweight helpers that encapsulate repetitive byte sequences required to command
 * the DUPLO Train Hub. These routines are leveraged by `DuploHub` to provide richer features while
 * keeping low-level device interaction centralized.
 *
 * @author Ralf Zühlsdorff
 * @date 2025
 *
 * @copyright
 * MIT License
 */

#include "myLegoHub.h"


/**
 * @brief Activate the base speaker port device.
 *
 * Activates the DUPLO speaker port and configures the device to accept sound commands.
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
 * Sends the byte sequence needed to start playback of the provided sound ID on the hub speaker.
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
 * Requests the DUPLO hub to expose its RGB LED port for color control commands.
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
 * Transmits the RGB color payload to the hub so the LED reflects the requested hue.
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
 * Normalizes raw sensor readings to a predictable range for application logic.
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
