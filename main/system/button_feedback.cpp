#include "button_feedback.h"
#include "led/led_controller.h"
#include "led/led_strip.h"
#include "esp_timer.h"

const char* ButtonFeedbackController::TAG = "ButtonFeedback";

ButtonFeedbackController::ButtonFeedbackController(LEDController* led_controller)
    : led_controller_(led_controller)
    , feedback_active_(false)
    , current_feedback_(BUTTON_PRESS_FEEDBACK)
    , feedback_start_time_(0)
    , feedback_duration_ms_(0)
    , pattern_saved_(false)
    , saved_pixels_(nullptr)
    , saved_brightness_(0)
    , led_count_(0)
{
}

ButtonFeedbackController::~ButtonFeedbackController() {
    // Restore pattern if feedback was active during destruction
    if (feedback_active_ && pattern_saved_) {
        restorePattern();
    }
}

esp_err_t ButtonFeedbackController::init() {
    if (!led_controller_) {
        ESP_LOGE(TAG, "Invalid LED controller - cannot initialize feedback");
        return ESP_ERR_INVALID_ARG;
    }

    feedback_active_ = false;
    pattern_saved_ = false;
    
    ESP_LOGI(TAG, "✅ ButtonFeedbackController initialized - Local-only LED feedback ready");
    return ESP_OK;
}

void ButtonFeedbackController::showFeedback(FeedbackType type) {
    if (!led_controller_) {
        ESP_LOGW(TAG, "LED controller not available for feedback");
        return;
    }
    
    // If feedback is already active, don't interrupt it
    if (feedback_active_) {
        ESP_LOGD(TAG, "Feedback already active - ignoring %s", getFeedbackTypeName(type));
        return;
    }

    // Use LED task-based feedback to prevent RMT timing conflicts on ESP32-C3
    uint32_t duration_ms = getFeedbackDuration(type);
    esp_err_t result = led_controller_->showButtonFeedback(type, duration_ms);
    
    if (result == ESP_OK) {
        // Set feedback as active for tracking
        current_feedback_ = type;
        feedback_start_time_ = esp_timer_get_time() / 1000;
        feedback_duration_ms_ = duration_ms;
        feedback_active_ = true;
        
        ESP_LOGD(TAG, "🎨 Button feedback sent to LED task: %s for %u ms", 
                 getFeedbackTypeName(type), feedback_duration_ms_);
    } else {
        ESP_LOGW(TAG, "Failed to send button feedback to LED task: %s", esp_err_to_name(result));
        
        // Fallback to direct access if LED task unavailable (compatibility mode)
        ESP_LOGW(TAG, "Falling back to direct LED strip access for button feedback");
        saveCurrentPattern();
        applyFeedbackPattern(type);
        
        current_feedback_ = type;
        feedback_start_time_ = esp_timer_get_time() / 1000;
        feedback_duration_ms_ = duration_ms;
        feedback_active_ = true;
    }
}

void ButtonFeedbackController::update() {
    if (!feedback_active_) {
        return;
    }
    
    uint32_t now = esp_timer_get_time() / 1000;
    uint32_t elapsed = now - feedback_start_time_;
    
    // Check if feedback duration has elapsed
    if (elapsed >= feedback_duration_ms_) {
        // Feedback complete - LED task handles its own cleanup, just mark as inactive
        feedback_active_ = false;
        
        ESP_LOGD(TAG, "🎨 %s feedback complete", getFeedbackTypeName(current_feedback_));
        
        // Only restore pattern if we used fallback direct access mode
        if (pattern_saved_) {
            restorePattern();
        }
        return;
    }
    
    // For LED task-based feedback, no main loop animation updates needed
    // The LED task handles all animation internally
    // Only call updateAnimatedPatterns if using fallback mode (pattern_saved_ == true)
    if (pattern_saved_) {
        updateAnimatedPatterns(elapsed);
    }
}

void ButtonFeedbackController::updateAnimatedPatterns(uint32_t elapsed_ms) {
    LEDStrip* strip = led_controller_->getLEDStrip();
    if (!strip) {
        return;
    }
    
    uint16_t led_count = strip->numPixels();
    if (led_count == 0) {
        return;
    }
    
    switch (current_feedback_) {
        case TAKEOVER_IN_PROGRESS: {
            // Purple marching pattern - shift every 150ms
            uint32_t phase = (elapsed_ms / 150) % 4;
            strip->clear();
            for (uint16_t i = phase; i < led_count; i += 4) {
                strip->setPixelColor(i, MAGENTA);
            }
            strip->show();
            break;
        }
        
        case TAKEOVER_FAILED: {
            // Red strobe - toggle every 100ms
            bool on = (elapsed_ms / 100) % 2 == 0;
            if (on) {
                for (uint16_t i = 0; i < led_count; i++) {
                    strip->setPixelColor(i, RED);
                }
            } else {
                strip->clear();
            }
            strip->show();
            break;
        }
        
        default:
            // No animation needed for other patterns
            break;
    }
}

void ButtonFeedbackController::saveCurrentPattern() {
    if (!led_controller_ || !led_controller_->isInitialized()) {
        ESP_LOGW(TAG, "Cannot save pattern - LED controller not initialized");
        return;
    }
    
    LEDStrip* strip = led_controller_->getLEDStrip();
    if (!strip) {
        ESP_LOGW(TAG, "Cannot save pattern - LED strip not available");
        return;
    }
    
    // Get LED count and allocate buffer for pixel state
    led_count_ = strip->numPixels();
    if (led_count_ == 0) {
        ESP_LOGW(TAG, "Cannot save pattern - no LEDs configured");
        return;
    }
    
    // Use new(std::nothrow) for ESP32-C3 compatibility (no exceptions)
    uint32_t* pixel_buffer = new(std::nothrow) uint32_t[led_count_];
    if (!pixel_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for pattern preservation");
        return;
    }
    saved_pixels_.reset(pixel_buffer);
    
    // Save current pixel colors
    for (uint16_t i = 0; i < led_count_; i++) {
        saved_pixels_[i] = strip->getPixelColor(i);
    }
    
    // Save current brightness
    saved_brightness_ = strip->getBrightness();
    pattern_saved_ = true;
    
    ESP_LOGD(TAG, "📸 LED pattern saved: %d pixels, brightness %d", led_count_, saved_brightness_);
}

void ButtonFeedbackController::restorePattern() {
    if (!pattern_saved_ || !saved_pixels_) {
        return;
    }
    
    LEDStrip* strip = led_controller_->getLEDStrip();
    if (!strip) {
        ESP_LOGW(TAG, "Cannot restore pattern - LED strip not available");
        pattern_saved_ = false;
        return;
    }
    
    // Restore pixel colors
    for (uint16_t i = 0; i < led_count_; i++) {
        strip->setPixelColor(i, saved_pixels_[i]);
    }
    
    // Restore brightness
    strip->setBrightness(saved_brightness_);
    
    // Show restored pattern
    strip->show();
    
    // Clean up saved state
    saved_pixels_.reset();
    pattern_saved_ = false;
    led_count_ = 0;
    
    ESP_LOGD(TAG, "🔄 LED pattern restored to previous state");
}

void ButtonFeedbackController::applyFeedbackPattern(FeedbackType type) {
    LEDStrip* strip = led_controller_->getLEDStrip();
    if (!strip) {
        ESP_LOGW(TAG, "Cannot apply feedback pattern - LED strip not available");
        return;
    }
    
    uint16_t led_count = strip->numPixels();
    if (led_count == 0) {
        ESP_LOGW(TAG, "Cannot apply feedback pattern - no LEDs configured");
        return;
    }
    
    // Apply LOCAL-ONLY feedback patterns that don't broadcast over mesh
    switch (type) {
        case BUTTON_PRESS_FEEDBACK: {
            // Quick white flash for button acknowledgment
            ESP_LOGD(TAG, "💡 Button press acknowledgment - quick white flash");
            strip->clear();
            // Light up first 5 LEDs with white
            for (uint16_t i = 0; i < 5 && i < led_count; i++) {
                strip->setPixelColor(i, WHITE);
            }
            strip->show();
            break;
        }
            
        case DUAL_PRESS_DETECTED: {
            // Blue flash indicating dual-button press detected
            ESP_LOGD(TAG, "🔵 Dual-button press detected - blue flash");
            strip->clear();
            // Fill strip with blue
            for (uint16_t i = 0; i < led_count; i++) {
                strip->setPixelColor(i, BLUE);
            }
            strip->show();
            break;
        }
            
        case TAKEOVER_IN_PROGRESS: {
            // Purple marching pattern for root takeover progress (animated in update())
            ESP_LOGD(TAG, "🟣 Root takeover in progress - purple march");
            // Initial purple pattern - will be animated in update()
            strip->clear();
            for (uint16_t i = 0; i < led_count; i += 4) {
                strip->setPixelColor(i, MAGENTA);  // Purple
            }
            strip->show();
            break;
        }
            
        case TAKEOVER_SUCCESS: {
            // Green flash for successful root takeover
            ESP_LOGD(TAG, "🟢 Root takeover success - green flash");
            strip->clear();
            // Fill strip with green
            for (uint16_t i = 0; i < led_count; i++) {
                strip->setPixelColor(i, GREEN);
            }
            strip->show();
            break;
        }
            
        case TAKEOVER_FAILED: {
            // Red strobe for failed root takeover (animated in update())
            ESP_LOGD(TAG, "🔴 Root takeover failed - red strobe");
            strip->clear();
            // Fill strip with red
            for (uint16_t i = 0; i < led_count; i++) {
                strip->setPixelColor(i, RED);
            }
            strip->show();
            break;
        }
            
        case BUTTON_BLOCKED_BLE: {
            // Yellow flash when buttons are blocked by BLE
            ESP_LOGD(TAG, "🟡 Buttons blocked by BLE - yellow flash");
            strip->clear();
            // Light up center LEDs with yellow
            uint16_t center = led_count / 2;
            for (uint16_t i = center > 2 ? center - 2 : 0; i < center + 3 && i < led_count; i++) {
                strip->setPixelColor(i, YELLOW);
            }
            strip->show();
            break;
        }
            
        case BUTTON_BLOCKED_CLIENT: {
            // Orange flash when buttons are blocked (not root)
            ESP_LOGD(TAG, "🟠 Buttons blocked (not root) - orange flash");
            strip->clear();
            // Light up center LEDs with orange (red + green)
            uint32_t orange = 0xFF8000;  // Orange color
            uint16_t center = led_count / 2;
            for (uint16_t i = center > 2 ? center - 2 : 0; i < center + 3 && i < led_count; i++) {
                strip->setPixelColor(i, orange);
            }
            strip->show();
            break;
        }
            
        default:
            ESP_LOGW(TAG, "Unknown feedback type: %d", type);
            break;
    }
}

uint32_t ButtonFeedbackController::getFeedbackDuration(FeedbackType type) const {
    switch (type) {
        case BUTTON_PRESS_FEEDBACK:
            return QUICK_FLASH_MS;
        case DUAL_PRESS_DETECTED:
            return DUAL_PRESS_FLASH_MS;
        case TAKEOVER_IN_PROGRESS:
            return TAKEOVER_MARCH_MS;
        case TAKEOVER_SUCCESS:
            return SUCCESS_FLASH_MS;
        case TAKEOVER_FAILED:
            return FAILURE_STROBE_MS;
        case BUTTON_BLOCKED_BLE:
        case BUTTON_BLOCKED_CLIENT:
            return BLOCKED_FLASH_MS;
        default:
            return QUICK_FLASH_MS;
    }
}

const char* ButtonFeedbackController::getFeedbackTypeName(FeedbackType type) const {
    switch (type) {
        case BUTTON_PRESS_FEEDBACK:   return "Button Press";
        case DUAL_PRESS_DETECTED:     return "Dual Press Detected";
        case TAKEOVER_IN_PROGRESS:    return "Takeover In Progress";
        case TAKEOVER_SUCCESS:        return "Takeover Success";
        case TAKEOVER_FAILED:         return "Takeover Failed";
        case BUTTON_BLOCKED_BLE:      return "Blocked by BLE";
        case BUTTON_BLOCKED_CLIENT:   return "Blocked (Not Root)";
        default:                      return "Unknown";
    }
}