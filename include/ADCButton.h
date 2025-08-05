
/**
 * @file ADCButton.h
 * @brief ADC-based button reading with averaging and debouncing for ESP32
 * 
 * This library provides a comprehensive solution for reading multiple buttons
 * connected via voltage divider to a single ADC pin. It features dynamic button
 * management, signal averaging, debouncing, and automatic calibration.
 * 
 * @author Ralf Zühlsdorff
 * @date 2025
 * @copyright Copyright (c) 2025 Ralf Zühlsdorff
 * 
 * @license MIT License
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

#pragma once
#include <Arduino.h>


typedef void (*buttonCallback)(int button_no, bool pressed);



/**
 * @class ADCButtons
 * @brief Multi-button ADC reader with averaging and debouncing
 * 
 * This class manages multiple buttons connected to a single ADC pin via voltage
 * divider circuits. It provides signal averaging, debouncing, and dynamic button
 * management with automatic memory allocation.
 */
class ADCButtons
{
private:
    /**
     * @struct ButtonState
     * @brief Structure to hold individual button state information
     */
    typedef struct
    {
        bool pressed;   ///< Current pressed state of the button
        int adcValue;   ///< Expected ADC value when button is pressed (readonly after setup)
    } ButtonState;

    // Configuration parameters
    int samples = 10;        ///< Number of ADC samples for averaging (default: 10)
    int margin = 50;         ///< Tolerance margin for ADC value comparison (default: 50)
    int resolution = 12;     ///< ADC resolution in bits (default: 12-bit = 4096 values)
    int debounceTime = 10;   ///< Debounce time in milliseconds (default: 10ms)
    int adcPin = 7;          ///< ADC pin number for button reading (default: pin 7)

    // ADC sampling and averaging
    int *values = nullptr;   ///< Circular buffer for ADC sample values (dynamically allocated)
    int min = 4095;          ///< Minimum ADC value recorded (for calibration)
    int max = 0;             ///< Maximum ADC value recorded (for calibration)

    // Dynamic button management
    ButtonState *buttons = nullptr;  ///< Dynamic array of button states
    int buttonCount = 0;             ///< Current number of registered buttons
    int buttonCapacity = 0;          ///< Current capacity of buttons array
    
    // Callback management
    buttonCallback callback = nullptr;  ///< Optional callback function for button state changes

public:
    /**
     * @brief Constructor - Initialize ADC button reader with configurable parameters
     * @param adc_pin ADC pin number to read from (default: 7)
     * @param samples Number of samples for averaging (default: 10)
     * @param margin Tolerance margin for button detection (default: 50)
     * @param resolution ADC resolution in bits (default: 12)
     * @param debounceTime Debounce delay in milliseconds (default: 25)
     */
    ADCButtons(int adc_pin, int samples = 10, int margin = 50, int resolution = 12, int debounceTime = 10);
    
    ~ADCButtons();

    /**
     * @brief Register a callback function for button state changes
     * @param cb Callback function that will be called when any button state changes
     *           Function signature: void callback(int button_no, bool pressed)
     *           - button_no: 0-based index of the button that changed
     *           - pressed: true if button was pressed, false if released
     */
    void registerButtonCallback(buttonCallback cb);


    /**
     * @brief Register a new button with its expected ADC value
     * @param button_no Button number (0-based index) to assign to this button
     * @param adcValue The ADC value expected when this button is pressed
     */
    void addButton(int button_no, int adcValue);
    
    /**
     * @brief Update all button states by reading and processing ADC values
     * Should be called regularly in the main loop
     */
    void updateButtons();
    
    /**
     * @brief Get the current state of a specific button
     * @param button_no Button index (0-based)
     * @return true if button is currently pressed, false otherwise
     */
    bool getButtonState(int button_no);

    // Utility and information methods
    
    /**
     * @brief Get the total number of registered buttons
     * @return Number of buttons currently registered
     */
    int getButtonCount() const;
    
    /**
     * @brief Get the highest button number currently in use
     * @return Highest button number (0-based), or -1 if no buttons registered
     */
    int getMaxButtonNumber() const;
    
    /**
     * @brief Get the current averaged ADC reading
     * @return Averaged ADC value from recent samples
     */
    int getAverageADC();
    
    /**
     * @brief Get the minimum ADC value recorded since last reset
     * @return Minimum ADC value (useful for calibration)
     */
    int getMinValue();
    
    /**
     * @brief Get the maximum ADC value recorded since last reset
     * @return Maximum ADC value (useful for calibration)
     */
    int getMaxValue();
    
    /**
     * @brief Get the configured ADC pin number
     * @return ADC pin number being used for button reading
     */
    int getAdcPin() const;
    
    /**
     * @brief Reset the min/max calibration values
     * Useful when recalibrating the system
     */
    void resetCalibration();
};