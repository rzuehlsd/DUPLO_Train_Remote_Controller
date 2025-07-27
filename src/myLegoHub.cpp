#include "myLegoHub.h"

void myLegoHub::activateBaseSpeaker()
{
    // Activate the speaker port device first
    activatePortDevice((byte)DuploTrainHubPort::SPEAKER);
    delay(100); // Give time for activation
    
    // Set sound mode 
    byte setSoundMode[8] = { 0x41, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01 };
    WriteValue(setSoundMode, 8);
}

void myLegoHub::playSound(byte sound)
{
    // Then play the sound
    byte playSound[6] = { 0x81, 0x01, 0x11, 0x51, 0x01, sound };
    WriteValue(playSound, 6);
}

void myLegoHub::activateRgbLight()
{
    byte port = getPortForDeviceType((byte)DeviceType::HUB_LED);
    byte setColorMode[8] = { 0x41, port, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 };
    WriteValue(setColorMode, 8);
}

void myLegoHub::setLedColor(Color color)
{
    byte port = getPortForDeviceType((byte)DeviceType::HUB_LED);
    byte setColor[6] = { 0x81, port, 0x11, 0x51, 0x00, color };
    Lpf2Hub::WriteValue(setColor, 6);
}



/**
 * @brief Map raw speedometer values to normalized -100..100 range
 * @param [in] rawSpeed Raw speedometer value from sensor (-32768..32767)
 * @param [in] maxRawSpeed Maximum raw speed value for mapping (default 255)
 * @return Normalized speed value (-100..100)
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
