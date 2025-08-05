#include <Arduino.h>
#include <FastLED.h>



class StatusLED
{
public:
    StatusLED(int pin);
    void begin(); // Add begin method to initialize FastLED
    void update();
    void setColor(CRGB color);
    void setBrightness(int val = 50);
    void setBlinking(bool blinking, int ontime = 200, int interval = 500, int times = -1);
    void setOff();

private:
    int _pin;
    CRGB _leds[1]; // FastLED array with one LED
    CRGB _color;   // Current color to display
    int _pattern;
    bool _blinking;
    int _interval;
    int _ontime;
    int _blinkTimes;      // Number of times to blink (-1 = infinite)
    int _currentBlinks;   // Counter for current blink cycles
    bool _lastWasOn;      // Track previous LED state for blink counting
    unsigned long _lastUpdate;
    bool _initialized;
};
