#include "system_control/root_takeover_manager.h"

const char* RootTakeoverManager::TAG = "RootTakeoverManager";

RootTakeoverManager::RootTakeoverManager()
    : current_state(INACTIVE)
    , state_start_time(0)
    , takeover_start_time(0)
    , mesh_coordinator(nullptr)
    , original_role(NodeRole::MESH_CLIENT)
    , waiting_for_role_change(false) {
    ESP_LOGI(TAG, "RootTakeoverManager created");
}

esp_err_t RootTakeoverManager::init(ESPNowMeshCoordinator* mesh_coordinator) {
    if (!mesh_coordinator) {
        ESP_LOGE(TAG, "❌ Mesh coordinator is null");
        return ESP_ERR_INVALID_ARG;
    }
    
    this->mesh_coordinator = mesh_coordinator;
    original_role = mesh_coordinator->getCurrentRole();
    
    ESP_LOGI(TAG, "✅ Root takeover manager initialized");
    ESP_LOGI(TAG, "   Current mesh role: %s", mesh_coordinator->getRoleString());
    
    return ESP_OK;
}

void RootTakeoverManager::setTakeoverCallback(std::function<void(TakeoverResult)> callback) {
    takeover_callback = callback;
    ESP_LOGD(TAG, "Takeover completion callback registered");
}

void RootTakeoverManager::setVisualFeedbackCallback(std::function<void(TakeoverState)> callback) {
    visual_feedback_callback = callback;
    ESP_LOGD(TAG, "Visual feedback callback registered");
}

void RootTakeoverManager::startTakeover() {
    if (current_state != INACTIVE) {
        ESP_LOGW(TAG, "Takeover already in progress - ignoring start request");
        return;
    }

    ESP_LOGI(TAG, "STARTING ROOT TAKEOVER PROCESS");

    // Check if there's an active BLE-connected on *this* node (higher priority)
    if (mesh_coordinator->isBleConnected()) {
        ESP_LOGW(TAG, "BLE is connected to this node - takeover blocked by mobile app priority");
        transitionToState(BLE_OVERRIDE_ACTIVE);
        completeTakeover(FAILED_BLE_PRIORITY);
        return;
    }

    // Check if ANY node in the network has BLE connection (root election redesign)
    if (mesh_coordinator->networkHasBleRoot()) {
        ESP_LOGW(TAG, "Button takeover blocked - BLE root exists in network");
        transitionToState(BLE_OVERRIDE_ACTIVE);
        completeTakeover(FAILED_BLE_PRIORITY);
        return;
    }

    // Store original role for potential rollback
    original_role = mesh_coordinator->getCurrentRole();
    takeover_start_time = esp_timer_get_time() / 1000;

    transitionToState(DUAL_PRESS_DETECTED);
    ESP_LOGI(TAG, "   Original role: %s", mesh_coordinator->getRoleString());
}

void RootTakeoverManager::cancelTakeover() {
    if (current_state == INACTIVE) {
        return;
    }
    
    ESP_LOGW(TAG, "🛑 CANCELLING ROOT TAKEOVER");
    ESP_LOGW(TAG, "   Reason: User released buttons during takeover");
    
    completeTakeover(CANCELLED_USER);
}

void RootTakeoverManager::update() {
    if (current_state == INACTIVE) {
        return;
    }
    
    // Global timeout check (defensive: only if we have a valid start time)
    if (takeover_start_time > 0 && isTimeoutElapsed(takeover_start_time, TAKEOVER_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "❌ Takeover timeout after %dms", TAKEOVER_TIMEOUT_MS);
        completeTakeover(FAILED_TIMEOUT);
        return;
    }
    
    switch (current_state) {
        case DUAL_PRESS_DETECTED:
            // Brief delay for user feedback, then start actual takeover
            if (isTimeoutElapsed(state_start_time, STABILIZATION_DELAY_MS)) {
                transitionToState(INITIATING_TAKEOVER);
            }
            break;
            
        case INITIATING_TAKEOVER:
            initiateMeshTakeover();
            transitionToState(WAITING_NETWORK_SYNC);
            break;
            
        case WAITING_NETWORK_SYNC:
            if (waiting_for_role_change) {
                // Waiting for mesh role change callback
                if (isTimeoutElapsed(state_start_time, NETWORK_SYNC_TIMEOUT_MS)) {
                    ESP_LOGE(TAG, "❌ Network sync timeout - mesh role change not received");
                    completeTakeover(FAILED_NETWORK_ERROR);
                }
            } else {
                // Check if we successfully became autonomous root
                checkNetworkSynchronization();
            }
            break;
            
        case TAKEOVER_COMPLETE:
        case TAKEOVER_FAILED:
        case BLE_OVERRIDE_ACTIVE:
            // Terminal states - reset to inactive after brief display period
            if (isTimeoutElapsed(state_start_time, 2000)) { // 2-second display
                transitionToState(INACTIVE);
            }
            break;
            
        case INACTIVE:
        default:
            // Should not reach here
            break;
    }
}

void RootTakeoverManager::onBleConnected() {
    if (isTakeoverActive()) {
        ESP_LOGW(TAG, "🔗 BLE connected during takeover - mobile app takes priority");
        transitionToState(BLE_OVERRIDE_ACTIVE);
        completeTakeover(FAILED_BLE_PRIORITY);
    }
}

void RootTakeoverManager::onMeshRoleChanged(NodeRole old_role, NodeRole new_role) {
    if (!waiting_for_role_change) {
        return;
    }
    
    ESP_LOGI(TAG, "🔄 Mesh role change: %s -> %s", 
             mesh_coordinator->getRoleString(), // This will show new role
             (new_role == NodeRole::MESH_ROOT_AUTONOMOUS) ? "AUTONOMOUS_ROOT" : "OTHER");
    
    waiting_for_role_change = false;
    
    if (current_state == WAITING_NETWORK_SYNC) {
        if (new_role == NodeRole::MESH_ROOT_AUTONOMOUS) {
            ESP_LOGI(TAG, "✅ Successfully became autonomous root!");
            finalizeTakeover();
        } else {
            ESP_LOGE(TAG, "❌ Failed to become autonomous root - unexpected role");
            completeTakeover(FAILED_NETWORK_ERROR);
        }
    }
}

void RootTakeoverManager::transitionToState(TakeoverState new_state) {
    if (current_state != new_state) {
        TakeoverState old_state = current_state;
        current_state = new_state;
        state_start_time = esp_timer_get_time() / 1000;
        
        // Reset takeover timing when returning to INACTIVE to prevent timeout loops
        if (new_state == INACTIVE) {
            takeover_start_time = 0;
            waiting_for_role_change = false;
            ESP_LOGI(TAG, "🔄 Takeover state reset - ready for new attempts");
        }
        
        ESP_LOGI(TAG, "State transition: %s -> %s", 
                 getStateString(old_state), getStateString(new_state));
        
        triggerVisualFeedback();
    }
}

bool RootTakeoverManager::isTimeoutElapsed(uint32_t start_time, uint32_t timeout_ms) {
    uint32_t current_time = esp_timer_get_time() / 1000;
    return (current_time - start_time) >= timeout_ms;
}

void RootTakeoverManager::completeTakeover(TakeoverResult result) {
    ESP_LOGI(TAG, "🏁 TAKEOVER COMPLETE: %s", getResultString(result));
    
    if (result == SUCCESS) {
        transitionToState(TAKEOVER_COMPLETE);
    } else {
        transitionToState(TAKEOVER_FAILED);
    }
    
    waiting_for_role_change = false;
    
    if (takeover_callback) {
        takeover_callback(result);
    }
}

void RootTakeoverManager::triggerVisualFeedback() {
    if (visual_feedback_callback) {
        visual_feedback_callback(current_state);
    }
}

void RootTakeoverManager::initiateMeshTakeover() {
    ESP_LOGI(TAG, "🌐 Initiating mesh coordinator takeover...");
    
    // Check if we're already autonomous root
    if (mesh_coordinator->getCurrentRole() == NodeRole::MESH_ROOT_AUTONOMOUS) {
        ESP_LOGI(TAG, "✅ Already autonomous root - takeover successful immediately");
        waiting_for_role_change = false;
        finalizeTakeover();
        return;
    }
    
    // Use the mesh coordinator's broadcast root claim for manual override
    waiting_for_role_change = true;
    mesh_coordinator->broadcastRootClaim(RootClaimReason::BUTTON_PRESS);
    
    ESP_LOGI(TAG, "   Button-triggered root claim broadcasted - waiting for role change");
}

void RootTakeoverManager::checkNetworkSynchronization() {
    // Verify that we are now the autonomous root
    if (mesh_coordinator->getCurrentRole() == NodeRole::MESH_ROOT_AUTONOMOUS) {
        ESP_LOGI(TAG, "✅ Confirmed autonomous root status");
        finalizeTakeover();
    } else {
        ESP_LOGD(TAG, "Still waiting for autonomous root role...");
    }
}

void RootTakeoverManager::finalizeTakeover() {
    ESP_LOGI(TAG, "🎯 Finalizing successful root takeover");
    ESP_LOGI(TAG, "   Node is now autonomous mesh root");
    ESP_LOGI(TAG, "   Network controlled via button interface");
    
    completeTakeover(SUCCESS);
}

const char* RootTakeoverManager::getStateString(TakeoverState state) const {
    switch (state) {
        case INACTIVE: return "INACTIVE";
        case DUAL_PRESS_DETECTED: return "DUAL_PRESS_DETECTED";
        case INITIATING_TAKEOVER: return "INITIATING_TAKEOVER";
        case WAITING_NETWORK_SYNC: return "WAITING_NETWORK_SYNC";
        case TAKEOVER_COMPLETE: return "TAKEOVER_COMPLETE";
        case TAKEOVER_FAILED: return "TAKEOVER_FAILED";
        case BLE_OVERRIDE_ACTIVE: return "BLE_OVERRIDE_ACTIVE";
        default: return "UNKNOWN";
    }
}

const char* RootTakeoverManager::getResultString(TakeoverResult result) const {
    switch (result) {
        case SUCCESS: return "SUCCESS";
        case FAILED_NETWORK_ERROR: return "FAILED_NETWORK_ERROR";
        case FAILED_BLE_PRIORITY: return "FAILED_BLE_PRIORITY";
        case FAILED_TIMEOUT: return "FAILED_TIMEOUT";
        case CANCELLED_USER: return "CANCELLED_USER";
        default: return "UNKNOWN";
    }
}
