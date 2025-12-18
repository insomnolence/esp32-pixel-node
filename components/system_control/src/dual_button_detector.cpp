#include "system_control/dual_button_detector.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

const char* DualButtonDetector::TAG = "DualButtonDetector";

DualButtonDetector::DualButtonDetector() 
    : current_state(IDLE)
    , state_start_time(0)
    , button1_press_time(0)
    , button2_press_time(0)
    , dual_press_start_time(0)
    , button1_currently_held(false)
    , button2_currently_held(false)
    , last_button_sample_time(0) {
    ESP_LOGI(TAG, "DualButtonDetector created");
}

esp_err_t DualButtonDetector::init() {
    ESP_LOGI(TAG, "🔄 Initializing dual button detection...");
    ESP_LOGI(TAG, "Detection window: %dms, Hold confirmation: %dms",
             (int)SIMULTANEOUS_WINDOW_MS, (int)HOLD_CONFIRMATION_MS);
    
    resetDetection();
    
    ESP_LOGI(TAG, "✅ Dual button detection initialized");
    return ESP_OK;
}

void DualButtonDetector::setDualButtonCallback(std::function<void(DualButtonEvent)> callback) {
    dual_button_callback = callback;
    ESP_LOGD(TAG, "Dual button event callback registered");
}

void DualButtonDetector::processButtonEvent(ButtonManager::ButtonEvent event) {
    uint32_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds
    
    ESP_LOGD(TAG, "Processing button event in state %s", getStateString(current_state));
    
    switch (current_state) {
        case IDLE:
            if (event == ButtonManager::BUTTON_1_PRESS) {
                button1_press_time = current_time;
                transitionToState(BUTTON_1_PENDING);
                ESP_LOGI(TAG, "🔘 Button 1 detected - waiting for Button 2 within %dms",
                         (int)SIMULTANEOUS_WINDOW_MS);
            } else if (event == ButtonManager::BUTTON_2_PRESS) {
                button2_press_time = current_time;
                transitionToState(BUTTON_2_PENDING);
                ESP_LOGI(TAG, "🔘 Button 2 detected - waiting for Button 1 within %dms",
                         (int)SIMULTANEOUS_WINDOW_MS);
            }
            break;
            
        case BUTTON_1_PENDING:
            if (event == ButtonManager::BUTTON_2_PRESS) {
                button2_press_time = current_time;
                uint32_t time_diff = current_time - button1_press_time;
                
                if (time_diff <= SIMULTANEOUS_WINDOW_MS) {
                    dual_press_start_time = current_time;
                    transitionToState(DUAL_DETECTED);
                    ESP_LOGI(TAG, "🎯 DUAL PRESS DETECTED! Time difference: %lums", (unsigned long)time_diff);
                    
                    if (dual_button_callback) {
                        dual_button_callback(DUAL_PRESS_DETECTED);
                    }
                } else {
                    ESP_LOGW(TAG, "Button 2 too late - time difference: %lums (max: %dms)",
                             (unsigned long)time_diff, (int)SIMULTANEOUS_WINDOW_MS);
                    resetDetection();
                }
            }
            break;
            
        case BUTTON_2_PENDING:
            if (event == ButtonManager::BUTTON_1_PRESS) {
                button1_press_time = current_time;
                uint32_t time_diff = current_time - button2_press_time;
                
                if (time_diff <= SIMULTANEOUS_WINDOW_MS) {
                    dual_press_start_time = current_time;
                    transitionToState(DUAL_DETECTED);
                    ESP_LOGI(TAG, "🎯 DUAL PRESS DETECTED! Time difference: %lums", (unsigned long)time_diff);
                    
                    if (dual_button_callback) {
                        dual_button_callback(DUAL_PRESS_DETECTED);
                    }
                } else {
                    ESP_LOGW(TAG, "Button 1 too late - time difference: %lums (max: %dms)",
                             (unsigned long)time_diff, (int)SIMULTANEOUS_WINDOW_MS);
                    resetDetection();
                }
            }
            break;
            
        case DUAL_DETECTED:
        case HOLD_CONFIRMATION:
            // Additional button presses during hold confirmation are ignored
            ESP_LOGD(TAG, "Ignoring button press during dual press hold confirmation");
            break;
    }
}

void DualButtonDetector::update() {
    uint32_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds
    
    // Sample button states periodically for hold detection
    if (current_time - last_button_sample_time >= BUTTON_SAMPLE_INTERVAL_MS) {
        sampleButtonStates();
        last_button_sample_time = current_time;
    }
    
    switch (current_state) {
        case BUTTON_1_PENDING:
        case BUTTON_2_PENDING:
            // Check for timeout in single button pending states
            if (isTimeoutElapsed(state_start_time, SIMULTANEOUS_WINDOW_MS)) {
                ESP_LOGD(TAG, "Single button timeout - returning to IDLE");
                resetDetection();
            }
            break;
            
        case DUAL_DETECTED:
            // Transition to hold confirmation state
            transitionToState(HOLD_CONFIRMATION);
            ESP_LOGI(TAG, "🕐 Hold confirmation started - need %dms hold", (int)HOLD_CONFIRMATION_MS);
            break;
            
        case HOLD_CONFIRMATION:
            // Check if both buttons are still held
            if (!areBothButtonsHeld()) {
                ESP_LOGW(TAG, "❌ Dual press cancelled - buttons released before confirmation");
                if (dual_button_callback) {
                    dual_button_callback(DUAL_PRESS_CANCELLED);
                }
                resetDetection();
            } else if (isTimeoutElapsed(dual_press_start_time, HOLD_CONFIRMATION_MS)) {
                ESP_LOGI(TAG, "✅ DUAL HOLD CONFIRMED! Triggering root takeover...");
                if (dual_button_callback) {
                    dual_button_callback(DUAL_HOLD_CONFIRMED);
                }
                resetDetection();
            }
            break;
            
        case IDLE:
        default:
            // No action needed in IDLE state
            break;
    }
}

void DualButtonDetector::transitionToState(DetectionState new_state) {
    if (current_state != new_state) {
        DetectionState old_state = current_state;
        current_state = new_state;
        state_start_time = esp_timer_get_time() / 1000;
        
        ESP_LOGD(TAG, "State transition: %s -> %s", 
                 getStateString(old_state), getStateString(new_state));
    }
}

bool DualButtonDetector::isTimeoutElapsed(uint32_t start_time, uint32_t timeout_ms) {
    uint32_t current_time = esp_timer_get_time() / 1000;
    return (current_time - start_time) >= timeout_ms;
}

void DualButtonDetector::sampleButtonStates() {
    // Read current GPIO states directly for hold detection
    button1_currently_held = (gpio_get_level((gpio_num_t)CONFIG_BUTTON_1_GPIO) == 0); // Active low
    button2_currently_held = (gpio_get_level((gpio_num_t)CONFIG_BUTTON_2_GPIO) == 0); // Active low
    
    ESP_LOGV(TAG, "Button states: B1=%s, B2=%s", 
             button1_currently_held ? "HELD" : "released",
             button2_currently_held ? "HELD" : "released");
}

bool DualButtonDetector::areBothButtonsHeld() {
    return button1_currently_held && button2_currently_held;
}

void DualButtonDetector::resetDetection() {
    current_state = IDLE;
    state_start_time = 0;
    button1_press_time = 0;
    button2_press_time = 0;
    dual_press_start_time = 0;
    button1_currently_held = false;
    button2_currently_held = false;
    ESP_LOGD(TAG, "Detection state reset to IDLE");
}

const char* DualButtonDetector::getStateString(DetectionState state) const {
    switch (state) {
        case IDLE: return "IDLE";
        case BUTTON_1_PENDING: return "BUTTON_1_PENDING";
        case BUTTON_2_PENDING: return "BUTTON_2_PENDING";
        case DUAL_DETECTED: return "DUAL_DETECTED";
        case HOLD_CONFIRMATION: return "HOLD_CONFIRMATION";
        default: return "UNKNOWN";
    }
}

const char* DualButtonDetector::getEventString(DualButtonEvent event) const {
    switch (event) {
        case DUAL_PRESS_DETECTED: return "DUAL_PRESS_DETECTED";
        case DUAL_HOLD_CONFIRMED: return "DUAL_HOLD_CONFIRMED";
        case DUAL_PRESS_CANCELLED: return "DUAL_PRESS_CANCELLED";
        default: return "UNKNOWN";
    }
}
