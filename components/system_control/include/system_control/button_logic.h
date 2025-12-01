#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system_control/button_manager.h"
#include "packet/packet.h"

// Forward declarations
class LEDController;
class ESPNowMeshCoordinator;
class DualButtonDetector;
class ButtonFeedbackController;

/**
 * @brief ButtonLogic handles business logic for button sequence management
 * 
 * This class implements the button state machine: Idle → Warning → Exit → Idle
 * and integrates with the dual-button root takeover system.
 * 
 * Architecture:
 * - Processes button events from ButtonManager
 * - Manages LED pattern state transitions
 * - Coordinates with mesh coordinator for root node operations
 * - Provides local-only feedback through ButtonFeedback controller
 */
class ButtonLogic {
public:
    /**
     * @brief Button state machine states
     */
    enum ButtonState {
        STATE_IDLE,      ///< Idle pattern, waiting for button press
        STATE_WARNING,   ///< Warning pattern active
        STATE_EXIT       ///< Exit pattern active
    };

    /**
     * @brief Constructor with dependency injection for testing
     * @param led_controller Pointer to LED controller for pattern management
     * @param mesh_coordinator Pointer to mesh coordinator for root operations
     * @param feedback_controller Pointer to button feedback controller for local LED feedback
     */
    ButtonLogic(LEDController* led_controller, ESPNowMeshCoordinator* mesh_coordinator, ButtonFeedbackController* feedback_controller);

    /**
     * @brief Destructor
     */
    ~ButtonLogic();

    /**
     * @brief Initialize button logic system
     * @return ESP_OK on success, error code on failure
     */
    esp_err_t init();

    /**
     * @brief Process button events from ButtonManager
     * @param event Button event to process
     */
    void handleButtonEvent(ButtonManager::ButtonEvent event);

    /**
     * @brief Update button logic state machine (called from main loop)
     */
    void update();

    /**
     * @brief Get current button state
     * @return Current button state
     */
    ButtonState getCurrentState() const { return current_state_; }

    /**
     * @brief Handle BLE connection status change
     * @param connected True if BLE is connected, false otherwise
     */
    void onBleConnectionChanged(bool connected);

    /**
     * @brief Handle mesh role change notifications
     * @param is_root True if this node is now root, false otherwise
     */
    void onMeshRoleChanged(bool is_root);

private:
    // Dependencies (injected for testability)
    LEDController* led_controller_;
    ESPNowMeshCoordinator* mesh_coordinator_;
    ButtonFeedbackController* feedback_controller_;

    // State management
    ButtonState current_state_;
    uint32_t last_state_change_;
    bool is_root_node_;
    bool ble_connected_;

    // Stack-safe member variables (no large allocations)
    static const uint32_t STATE_TIMEOUT_MS = 30000; ///< 30 second state timeout
    static const char* TAG; ///< Logging tag

    /**
     * @brief Process Button 1 press (state machine progression)
     */
    void handleButton1Press();

    /**
     * @brief Process Button 2 press (random pattern selection)
     */
    void handleButton2Press();

    /**
     * @brief Check if buttons should be ignored (not root node)
     * @return True if buttons should be ignored
     */
    bool shouldIgnoreButtons() const;

    /**
     * @brief Transition to new button state
     * @param new_state Target state to transition to
     */
    void transitionToState(ButtonState new_state);

    /**
     * @brief Check for state timeouts and handle recovery
     */
    void checkStateTimeout();
};
