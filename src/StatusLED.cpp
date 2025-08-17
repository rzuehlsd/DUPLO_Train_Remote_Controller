
#include "StatusLED.h"

StatusLED::StatusLED(int pin) : _pin(pin), _color(CRGB::Black), _pattern(0), _blinking(false), _interval(500), _ontime(200), _blinkTimes(-1), _currentBlinks(0), _lastWasOn(false), _lastUpdate(0), _initialized(false)
{
    _leds[0] = CRGB::Black;
}

/**
 * @brief Initialize the StatusLED with FastLED
 * Call this method in your setup() function
 */
void StatusLED::begin()
{
    if (!_initialized)
    {
        // Initialize FastLED for the specific pin
        // Note: Pin must be known at compile time for FastLED templates
        // Common pins: 48 for ESP32-S3-DevKitC-1
        switch (_pin)
        {
            case 48:
                FastLED.addLeds<NEOPIXEL, 48>(_leds, 1);
                break;
            case 2:
                FastLED.addLeds<NEOPIXEL, 2>(_leds, 1);
                break;
            case 8:
                FastLED.addLeds<NEOPIXEL, 8>(_leds, 1);
                break;
            default:
                // Default to pin 48 if pin not specifically supported
                FastLED.addLeds<NEOPIXEL, 48>(_leds, 1);
                break;
        }
        FastLED.setBrightness(50);
        _initialized = true;
    }
    
    _leds[0] = _color;
    FastLED.show();
}

/**
 * @brief Set the LED color
 * @param color CRGB color value to set
 */
void StatusLED::setColor(CRGB color)
{
    _color = color;
    _blinking = false;
    if (!_blinking && _initialized)
    {
        _leds[0] = _color;
        FastLED.show();
    }
}

/**
 * @brief Set the LED brightness
 * @param val Brightness value (0-255)
 */
void StatusLED::setBrightness(int val)
{
    if (_initialized)
    {
        FastLED.setBrightness(val);
        FastLED.show();
    }
}

void StatusLED::setOff()
{
    _color = CRGB::Black;
    _blinking = false;
    if (_initialized)
    {
        _leds[0] = CRGB::Black;
        FastLED.show();
    }
}


/**
 * @brief Update the LED state - call this regularly in your main loop
 * Handles blinking patterns if enabled
 */
void StatusLED::update()
{
    if (!_initialized)
    {
        return; // Not initialized yet
    }
    
    if (!_blinking)
    {
        // If not blinking, just ensure the LED shows the current color
        _leds[0] = _color;
        FastLED.show();
        return;
    }
    
    // Check if finite blinking is complete
    if (_blinkTimes > 0 && _currentBlinks >= _blinkTimes)
    {
        // Finite blinking is complete, stop blinking and turn LED off
        _blinking = false;
        _color = CRGB::Black; // Also reset color to black
        _leds[0] = CRGB::Black; // Turn LED off after finite blinks
        FastLED.show();
        return;
    }
    
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - _lastUpdate;
    
    // Calculate the position in the blink cycle
    unsigned long cycleTime = _ontime + _interval;
    unsigned long cyclePosition = elapsed % cycleTime;
    
    // Check if we've completed a full cycle to increment blink counter
    bool currentlyOn = (cyclePosition < _ontime);
    
    // Detect transition from off to on (start of new blink)
    if (currentlyOn && !_lastWasOn && _blinkTimes > 0)
    {
        _currentBlinks++;
    }
    _lastWasOn = currentlyOn;
    
    if (currentlyOn)
    {
        // LED should be on (showing the color)
        _leds[0] = _color;
    }
    else
    {
        // LED should be off
        _leds[0] = CRGB::Black;
    }
    
    FastLED.show();
}

/**
 * @brief Enable or disable blinking mode
 * @param blinking True to enable blinking, false to disable
 * @param ontime Duration in milliseconds the LED is on during each blink cycle
 * @param interval Duration in milliseconds the LED is off during each blink cycle
 * @param times Number of times to blink (-1 = infinite, >0 = finite)
 */
void StatusLED::setBlinking(bool blinking, int ontime, int interval, int times)
{
    _blinking = blinking;
    _ontime = ontime;
    _interval = interval;
    _blinkTimes = times;
    _currentBlinks = 0;  // Reset blink counter
    _lastWasOn = false;  // Reset state tracking
    _lastUpdate = millis();
    
    if (!_blinking && _initialized)
    {
        // If blinking is disabled, set LED to solid color
        _leds[0] = _color;
        FastLED.show();
    }
}


