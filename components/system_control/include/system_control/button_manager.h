#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "system_control/system_hardware_interface.h"
#include <functional>

/**
 * @brief Button Manager for ESP32 LED Mesh Button Interface
 * 
 * Handles GPIO setup, interrupt-based button detection, and debouncing.
 * Platform-configurable for ESP32C3 custom board and other ESP32 variants.
 */
class ButtonManager {
public:
    enum ButtonEvent {
        BUTTON_1_PRESS,    // GPIO configured as BUTTON_1_GPIO (default: GPIO 3 on ESP32C3)
        BUTTON_2_PRESS     // GPIO configured as BUTTON_2_GPIO (default: GPIO 2 on ESP32C3)
    };
    
    ButtonManager(SystemHardwareInterface* hardware);
    
    /**
     * @brief Initialize GPIO pins and interrupt handlers
     * @param pin1_gpio GPIO number for Button 1 (Sequence)
     * @param pin2_gpio GPIO number for Button 2 (Random)
     * @param debounce_ms Debounce time in milliseconds
     * @return ESP_OK on success, error code on failure
     */
    esp_err_t init(uint8_t pin1_gpio, uint8_t pin2_gpio, uint32_t debounce_ms);
    
    /**
     * @brief Set callback function for button events
     * @param callback Function to call when button is pressed
     */
    void setEventCallback(std::function<void(ButtonEvent)> callback);
    
    /**
     * @brief Process button events (call from main loop)
     * Handles debouncing and triggers callbacks for valid button presses
     * Uses GPIO polling to avoid RMT timing conflicts on ESP32-C3
     */
    void processEvents();
    
private:
    // Button state tracking (polling-based, no interrupts)
    bool button1_last_state = true;  // Buttons are active-low (pulled up)
    bool button2_last_state = true;
    uint32_t button1_last_time = 0;
    uint32_t button2_last_time = 0;
    
    // Configuration
    uint8_t pin1;
    uint8_t pin2;
    uint32_t debounce_delay;
    
    static const char* TAG;
    SystemHardwareInterface* hardware;
    std::function<void(ButtonEvent)> eventCallback;
    
    // Helper methods
    bool isDebounceTimeElapsed(uint32_t last_time);
    void handleButtonPress(ButtonEvent event, uint32_t* last_time);
};