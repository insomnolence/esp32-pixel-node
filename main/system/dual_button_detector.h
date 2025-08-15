#pragma once

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "button_manager.h"
#include <functional>

/**
 * @brief Dual Button Detector for ESP32 LED Mesh Root Takeover
 * 
 * Detects simultaneous button presses to trigger mesh coordinator root takeover.
 * Implements precise timing requirements:
 * - 500ms window for simultaneous button detection
 * - 2-second hold confirmation for root takeover
 * 
 * Integrates with ButtonManager for button events and ESPNowMeshCoordinator for root control.
 */
class DualButtonDetector {
public:
    enum DetectionState {
        IDLE,                    // No button activity
        BUTTON_1_PENDING,        // Button 1 pressed, waiting for Button 2
        BUTTON_2_PENDING,        // Button 2 pressed, waiting for Button 1
        DUAL_DETECTED,           // Both buttons detected within window
        HOLD_CONFIRMATION        // Confirming 2-second hold
    };

    enum DualButtonEvent {
        DUAL_PRESS_DETECTED,     // Both buttons pressed within 500ms window
        DUAL_HOLD_CONFIRMED,     // 2-second hold confirmed - trigger root takeover
        DUAL_PRESS_CANCELLED     // Hold released before 2-second confirmation
    };

    DualButtonDetector();
    
    /**
     * @brief Initialize dual button detection
     * @return ESP_OK on success, error code on failure
     */
    esp_err_t init();
    
    /**
     * @brief Process button events from ButtonManager
     * @param event Button event (BUTTON_1_PRESS or BUTTON_2_PRESS)
     * 
     * Call this from the ButtonManager event callback to detect dual presses.
     */
    void processButtonEvent(ButtonManager::ButtonEvent event);
    
    /**
     * @brief Set callback function for dual button events
     * @param callback Function to call when dual button events occur
     */
    void setDualButtonCallback(std::function<void(DualButtonEvent)> callback);
    
    /**
     * @brief Update dual button state machine (call from main loop)
     * 
     * Handles timing validation, hold confirmation, and timeout management.
     */
    void update();
    
    /**
     * @brief Get current detection state
     * @return Current DetectionState
     */
    DetectionState getCurrentState() const { return current_state; }
    
    /**
     * @brief Check if dual press is currently active
     * @return true if in DUAL_DETECTED or HOLD_CONFIRMATION state
     */
    bool isDualPressActive() const {
        return current_state == DUAL_DETECTED || current_state == HOLD_CONFIRMATION;
    }

private:
    static const char* TAG;
    
    // Timing configuration
    static const uint32_t SIMULTANEOUS_WINDOW_MS = 500;     // 500ms window for simultaneous detection
    static const uint32_t HOLD_CONFIRMATION_MS = 2000;      // 2-second hold confirmation
    static const uint32_t BUTTON_SAMPLE_INTERVAL_MS = 50;   // Button state sampling rate
    
    // State management
    DetectionState current_state;
    uint32_t state_start_time;
    uint32_t button1_press_time;
    uint32_t button2_press_time;
    uint32_t dual_press_start_time;
    
    // Button state tracking for hold detection
    bool button1_currently_held;
    bool button2_currently_held;
    uint32_t last_button_sample_time;
    
    // Callback
    std::function<void(DualButtonEvent)> dual_button_callback;
    
    // Helper methods
    void transitionToState(DetectionState new_state);
    bool isTimeoutElapsed(uint32_t start_time, uint32_t timeout_ms);
    void sampleButtonStates();
    bool areBothButtonsHeld();
    void resetDetection();
    
    // Logging helpers
    const char* getStateString(DetectionState state) const;
    const char* getEventString(DualButtonEvent event) const;
};