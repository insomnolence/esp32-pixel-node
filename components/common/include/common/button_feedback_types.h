#pragma once

/**
 * @brief Button feedback types shared between ButtonFeedbackController and LEDController
 * 
 * This header contains only the enum definition to avoid circular dependencies
 * between button_feedback.h and led_controller.h
 */

enum FeedbackType {
    BUTTON_PRESS_FEEDBACK,      ///< Quick flash for button press acknowledgment
    DUAL_PRESS_DETECTED,        ///< Blue flash for dual-button press detection
    TAKEOVER_IN_PROGRESS,       ///< Purple march for root takeover in progress
    TAKEOVER_SUCCESS,           ///< Green flash for successful root takeover
    TAKEOVER_FAILED,            ///< Red strobe for failed root takeover
    BUTTON_BLOCKED_BLE,         ///< Yellow flash when buttons blocked by BLE
    BUTTON_BLOCKED_CLIENT       ///< Orange flash when buttons blocked (not root)
};