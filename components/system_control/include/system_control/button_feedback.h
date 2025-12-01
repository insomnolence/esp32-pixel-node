#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "common/button_feedback_types.h"
#include <memory>

// Forward declarations
class LEDController;
class ILedStrip;

/**
 * @brief ButtonFeedbackController provides LOCAL-ONLY LED feedback for button interface
 * 
 * This controller handles visual feedback patterns that should NOT be broadcast over the mesh.
 * It provides temporary visual feedback for button presses, dual-button detection, and root
 * takeover operations while preserving the original LED pattern state.
 * 
 * Key Features:
 * - Local-only feedback (no mesh broadcasting)
 * - Pattern state preservation and restoration
 * - Non-blocking feedback operations
 * - ESP32-C3 memory optimized
 */
class ButtonFeedbackController {
public:

    /**
     * @brief Constructor with dependency injection
     * @param led_controller Pointer to LED controller for direct LED access
     */
    explicit ButtonFeedbackController(LEDController* led_controller);

    /**
     * @brief Destructor
     */
    ~ButtonFeedbackController();

    /**
     * @brief Initialize button feedback controller
     * @return ESP_OK on success, error code on failure
     */
    esp_err_t init();

    /**
     * @brief Show button feedback pattern (non-blocking)
     * @param type Type of feedback to display
     */
    void showFeedback(FeedbackType type);

    /**
     * @brief Update feedback controller (called from main loop)
     * Handles timing for feedback patterns and restoration
     */
    void update();

    /**
     * @brief Check if feedback is currently active
     * @return True if feedback pattern is playing
     */
    bool isFeedbackActive() const { return feedback_active_; }

private:
    // Dependencies
    LEDController* led_controller_;

    // Feedback state management
    bool feedback_active_;
    FeedbackType current_feedback_;
    uint32_t feedback_start_time_;
    uint32_t feedback_duration_ms_;
    
    // Pattern preservation (stack-safe, minimal memory)
    bool pattern_saved_;
    std::unique_ptr<uint32_t[]> saved_pixels_;  ///< Saved LED pixel state
    uint8_t saved_brightness_;                  ///< Saved brightness level
    uint16_t led_count_;                        ///< Number of LEDs to preserve
    
    // Timing constants
    static const uint32_t QUICK_FLASH_MS = 200;      ///< Quick acknowledgment flash
    static const uint32_t DUAL_PRESS_FLASH_MS = 500; ///< Dual-press detection flash
    static const uint32_t TAKEOVER_MARCH_MS = 2000;  ///< Root takeover progress march
    static const uint32_t SUCCESS_FLASH_MS = 1000;   ///< Success confirmation flash
    static const uint32_t FAILURE_STROBE_MS = 1500;  ///< Failure indication strobe
    static const uint32_t BLOCKED_FLASH_MS = 300;    ///< Button blocked indication
    
    static const char* TAG; ///< Logging tag

    /**
     * @brief Save current LED pattern state for restoration
     */
    void saveCurrentPattern();

    /**
     * @brief Restore previously saved LED pattern
     */
    void restorePattern();

    /**
     * @brief Apply local-only feedback pattern to LEDs
     * @param type Feedback type to apply
     */
    void applyFeedbackPattern(FeedbackType type);

    /**
     * @brief Get duration for feedback type
     * @param type Feedback type
     * @return Duration in milliseconds
     */
    uint32_t getFeedbackDuration(FeedbackType type) const;

    /**
     * @brief Get feedback type name for logging
     * @param type Feedback type
     * @return Human-readable name
     */
    const char* getFeedbackTypeName(FeedbackType type) const;

    /**
     * @brief Update animated feedback patterns
     * @param elapsed_ms Milliseconds elapsed since feedback started
     */
    void updateAnimatedPatterns(uint32_t elapsed_ms);
};