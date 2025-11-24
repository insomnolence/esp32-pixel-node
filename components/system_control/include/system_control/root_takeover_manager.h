#pragma once

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "system_control/dual_button_detector.h"
#include "mesh/espnow_mesh_coordinator.h"
#include <functional>

/**
 * @brief Root Takeover Manager for ESP32 LED Mesh Network
 * 
 * Manages the process of taking over as mesh root coordinator through dual-button activation.
 * Provides state machine for safe root transition and integration with existing mesh system.
 * 
 * Features:
 * - Safe root takeover with network stabilization
 * - Visual feedback during takeover process
 * - BLE priority management (mobile app overrides)
 * - Automatic fallback on failure
 */
class RootTakeoverManager {
public:
    enum TakeoverState {
        INACTIVE,                // Not attempting takeover
        DUAL_PRESS_DETECTED,     // Dual button press detected, preparing takeover
        INITIATING_TAKEOVER,     // Starting root takeover process
        WAITING_NETWORK_SYNC,    // Waiting for mesh network synchronization
        TAKEOVER_COMPLETE,       // Successfully became autonomous root
        TAKEOVER_FAILED,         // Takeover failed, returning to normal operation
        BLE_OVERRIDE_ACTIVE      // BLE connection took priority over button control
    };

    enum TakeoverResult {
        SUCCESS,                 // Successfully became autonomous root
        FAILED_NETWORK_ERROR,    // Network communication failure
        FAILED_BLE_PRIORITY,     // BLE connection took priority
        FAILED_TIMEOUT,          // Takeover process timed out
        CANCELLED_USER           // User cancelled (released buttons)
    };

    RootTakeoverManager();
    
    /**
     * @brief Initialize root takeover manager
     * @param mesh_coordinator Reference to mesh coordinator for root control
     * @return ESP_OK on success, error code on failure
     */
    esp_err_t init(ESPNowMeshCoordinator* mesh_coordinator);
    
    /**
     * @brief Start root takeover process from dual button detection
     * 
     * Called when dual button hold is confirmed. Initiates the root takeover sequence.
     */
    void startTakeover();
    
    /**
     * @brief Cancel ongoing takeover process
     * 
     * Called when buttons are released during takeover or on timeout.
     */
    void cancelTakeover();
    
    /**
     * @brief Update takeover state machine (call from main loop)
     * 
     * Manages takeover process, timeouts, and state transitions.
     */
    void update();
    
    /**
     * @brief Set callback for takeover completion
     * @param callback Function called when takeover completes (success or failure)
     */
    void setTakeoverCallback(std::function<void(TakeoverResult)> callback);
    
    /**
     * @brief Set callback for visual feedback updates
     * @param callback Function called for LED visual feedback during takeover
     */
    void setVisualFeedbackCallback(std::function<void(TakeoverState)> callback);
    
    /**
     * @brief Get current takeover state
     * @return Current TakeoverState
     */
    TakeoverState getCurrentState() const { return current_state; }
    
    /**
     * @brief Check if takeover is currently in progress
     * @return true if takeover is active
     */
    bool isTakeoverActive() const {
        return current_state != INACTIVE && current_state != TAKEOVER_COMPLETE && 
               current_state != TAKEOVER_FAILED;
    }
    
    /**
     * @brief Handle BLE connection events
     * 
     * Called when BLE connects during takeover to handle priority override.
     */
    void onBleConnected();
    
    /**
     * @brief Handle mesh role change events
     * @param old_role Previous mesh role
     * @param new_role New mesh role
     * 
     * Called when mesh coordinator role changes to track takeover success.
     */
    void onMeshRoleChanged(NodeRole old_role, NodeRole new_role);

private:
    static const char* TAG;
    
    // Timing configuration
    static const uint32_t TAKEOVER_TIMEOUT_MS = 15000;      // 15-second total timeout
    static const uint32_t NETWORK_SYNC_TIMEOUT_MS = 10000;  // 10-second network sync timeout  
    static const uint32_t STABILIZATION_DELAY_MS = 1000;    // 1-second stabilization delay
    
    // State management
    TakeoverState current_state;
    uint32_t state_start_time;
    uint32_t takeover_start_time;
    
    // Mesh integration
    ESPNowMeshCoordinator* mesh_coordinator;
    NodeRole original_role;
    bool waiting_for_role_change;
    
    // Callbacks
    std::function<void(TakeoverResult)> takeover_callback;
    std::function<void(TakeoverState)> visual_feedback_callback;
    
    // Helper methods
    void transitionToState(TakeoverState new_state);
    bool isTimeoutElapsed(uint32_t start_time, uint32_t timeout_ms);
    void completeTakeover(TakeoverResult result);
    void triggerVisualFeedback();
    
    // Takeover process steps
    void initiateMeshTakeover();
    void checkNetworkSynchronization();
    void finalizeTakeover();
    
    // Logging helpers
    const char* getStateString(TakeoverState state) const;
    const char* getResultString(TakeoverResult result) const;
};
