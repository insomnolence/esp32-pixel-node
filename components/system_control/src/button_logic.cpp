#include "system_control/button_logic.h"
#include "led/led_controller.h"
#include "mesh/espnow_mesh_coordinator.h"
#include "system_control/button_feedback.h"
#include "common/button_feedback_types.h"
#include "system_control/button_manager.h"
#include "esp_timer.h"

const char* ButtonLogic::TAG = "ButtonLogic";

ButtonLogic::ButtonLogic(LEDController* led_controller, ESPNowMeshCoordinator* mesh_coordinator, ButtonFeedbackController* feedback_controller)
    : led_controller_(led_controller)
    , mesh_coordinator_(mesh_coordinator)
    , feedback_controller_(feedback_controller)
    , current_state_(STATE_IDLE)
    , last_state_change_(0)
    , is_root_node_(false)
    , ble_connected_(false)
{
}

ButtonLogic::~ButtonLogic() {
    // No cleanup needed for stack-allocated members
}

esp_err_t ButtonLogic::init() {
    if (!led_controller_ || !mesh_coordinator_ || !feedback_controller_) {
        ESP_LOGE(TAG, "Invalid dependencies - LED controller, mesh coordinator, or feedback controller is null");
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize state
    current_state_ = STATE_IDLE;
    last_state_change_ = esp_timer_get_time() / 1000;
    is_root_node_ = mesh_coordinator_->isRootNode();
    
    ESP_LOGI(TAG, "✅ ButtonLogic initialized - Initial state: IDLE, Root: %s", 
             is_root_node_ ? "YES" : "NO");
    
    return ESP_OK;
}

void ButtonLogic::handleButtonEvent(ButtonManager::ButtonEvent event) {
    switch (event) {
        case ButtonManager::BUTTON_1_PRESS:
            handleButton1Press();
            break;
        case ButtonManager::BUTTON_2_PRESS:
            handleButton2Press();
            break;
        default:
            ESP_LOGW(TAG, "Unknown button event: %d", event);
            break;
    }
}

void ButtonLogic::update() {
    // Update feedback controller for timing
    feedback_controller_->update();
    
    // Check for state timeouts
    checkStateTimeout();
    
    // Update root node status
    bool current_root_status = mesh_coordinator_->isRootNode();
    if (current_root_status != is_root_node_) {
        onMeshRoleChanged(current_root_status);
    }
}

void ButtonLogic::onBleConnectionChanged(bool connected) {
    ble_connected_ = connected;
    ESP_LOGI(TAG, "BLE connection status changed: %s", connected ? "CONNECTED" : "DISCONNECTED");
    
    if (connected) {
        // BLE has priority - reset button state to idle
        transitionToState(STATE_IDLE);
        ESP_LOGI(TAG, "🔴 Button state reset to IDLE due to BLE connection (BLE has priority)");
    }
}

void ButtonLogic::onMeshRoleChanged(bool is_root) {
    bool previous_root = is_root_node_;
    is_root_node_ = is_root;
    
    ESP_LOGI(TAG, "Mesh role changed: %s → %s", 
             previous_root ? "ROOT" : "CLIENT", 
             is_root ? "ROOT" : "CLIENT");
    
    if (!is_root && previous_root) {
        // Lost root status - reset button state and LED pattern to idle
        // This allows the node to start receiving patterns from the new root
        transitionToState(STATE_IDLE);
        led_controller_->setIdleMode();
        ESP_LOGI(TAG, "🔴 Lost root status - LED pattern reset to IDLE, ready to receive from new root");
    }

    if (is_root && !previous_root) {
        // Gained root status - broadcast idle pattern to synchronize network
        transitionToState(STATE_IDLE);
        led_controller_->setIdleMode();
        ESP_LOGI(TAG, "🟢 Button control gained root - broadcasting idle pattern to network");
    }
}

void ButtonLogic::handleButton1Press() {
    // First check if buttons should be ignored
    if (shouldIgnoreButtons()) {
        ESP_LOGI(TAG, "🔴 Button 1 ignored - not root node (use dual-button press to take control)");
        // Silent blocking - no visual feedback to avoid pattern interruption
        return;
    }

    // Show button press acknowledgment feedback
    feedback_controller_->showFeedback(BUTTON_PRESS_FEEDBACK);

    // Process Button 1 state machine: Idle → Warning → Exit → Idle
    switch (current_state_) {
        case STATE_IDLE:
            ESP_LOGI(TAG, "🔴 Button 1: Idle → Warning (root control)");
            transitionToState(STATE_WARNING);
            led_controller_->setWarningMode();
            break;

        case STATE_WARNING:
            ESP_LOGI(TAG, "🔴 Button 1: Warning → Exit (root control)");
            transitionToState(STATE_EXIT);
            led_controller_->setExitMode();
            break;

        case STATE_EXIT:
            ESP_LOGI(TAG, "🔴 Button 1: Exit → Idle (root control)");
            transitionToState(STATE_IDLE);
            led_controller_->setIdleMode();
            break;

        default:
            ESP_LOGW(TAG, "Button 1 pressed in unknown state: %d", current_state_);
            transitionToState(STATE_IDLE);
            led_controller_->setIdleMode();
            break;
    }
}

void ButtonLogic::handleButton2Press() {
    // First check if buttons should be ignored
    if (shouldIgnoreButtons()) {
        ESP_LOGI(TAG, "🔵 Button 2 ignored - not root node (use dual-button press to take control)");
        // Silent blocking - no visual feedback to avoid pattern interruption
        return;
    }

    // Show button press acknowledgment feedback
    feedback_controller_->showFeedback(BUTTON_PRESS_FEEDBACK);

    ESP_LOGI(TAG, "🔵 Button 2: Random pattern selected (root control)");
    led_controller_->setRandomModeWithNewPattern();

    // Reset Button 1 state machine back to IDLE
    // This means the next Button 1 press will start Warning again (not continue to Exit)
    transitionToState(STATE_IDLE);
    ESP_LOGI(TAG, "🔵 Button 1 state reset to IDLE (Button 2 interrupt)");
}

bool ButtonLogic::shouldIgnoreButtons() const {
    // Buttons are ignored when:
    // 1. This node is not the root node (another node controls the network)
    // 2. BLE is connected (mobile app has priority over buttons)
    if (ble_connected_) {
        return true;  // BLE has priority - ignore buttons
    }
    return !is_root_node_;
}

void ButtonLogic::transitionToState(ButtonState new_state) {
    if (new_state != current_state_) {
        ButtonState old_state = current_state_;
        current_state_ = new_state;
        last_state_change_ = esp_timer_get_time() / 1000;
        
        ESP_LOGD(TAG, "State transition: %d → %d", old_state, new_state);
    }
}

void ButtonLogic::checkStateTimeout() {
    uint32_t now = esp_timer_get_time() / 1000;

    // Check for state timeout (recovery mechanism)
    // Skip timeout for WARNING and EXIT states - they have long-running sequences
    // that should play to completion (Flash→March→MiniTwinkle→Gradient)
    if (current_state_ == STATE_WARNING || current_state_ == STATE_EXIT) {
        return; // Let the sequence run to completion
    }

    if (now - last_state_change_ > STATE_TIMEOUT_MS) {
        if (current_state_ != STATE_IDLE) {
            ESP_LOGW(TAG, "State timeout after %lu ms - returning to IDLE", (unsigned long)STATE_TIMEOUT_MS);
            transitionToState(STATE_IDLE);
            led_controller_->setIdleMode();
        }
    }
}
