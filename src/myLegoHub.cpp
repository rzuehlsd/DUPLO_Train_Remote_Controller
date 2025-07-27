#include "myLegoHub.h"

void myLegoHub::activateBaseSpeaker()
{
    byte setSoundMode[8] = { 0x41, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01 };
    WriteValue(setSoundMode, 8);
}

void myLegoHub::playSound(byte sound)
{
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