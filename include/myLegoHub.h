#ifndef MY_LEGO_HUB_H
#define MY_LEGO_HUB_H

#include "Lpf2Hub.h"
#include "LegoinoCommon.h"

class myLegoHub : public Lpf2Hub {
public:
    void activateBaseSpeaker();
    void playSound(byte sound);
    void activateRgbLight();
    void setLedColor(Color color);
};

#endif // MY_LEGO_HUB_H
