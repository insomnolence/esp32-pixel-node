#include "bluetooth/ble_gatt_server.h"
#include "bluetooth/ble_gap_handler.h"
#include "bluetooth/gatt_profile.h"
#include "system/nvs_manager.h"
#include "system/network_health.h"
#include "bluetooth/pixel_packet_profile.h"
#include "bluetooth/network_health_profile.h"
#include "mesh/espnow_mesh_coordinator.h"
#include "packet/led_packet_processor.h"
#include "packet/generic_packet.h"
#include "led/led_controller.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_random.h"

#include <functional>
#include <memory>

#define MAIN_TAG "ESP_LED_MESH"
#define DEVICE_NAME "ESP_LED_NODE"


extern "C" void app_main(void) {
    // Enable debug logging for neighbor discovery
    esp_log_level_set("NeighborManager", ESP_LOG_DEBUG);
    esp_log_level_set("ESPNowMeshCoordinator", ESP_LOG_DEBUG);
    esp_log_level_set("NetworkHealthMonitor", ESP_LOG_DEBUG);
    ESP_LOGI(MAIN_TAG, "🔍 Enabled debug logging for mesh neighbor discovery components");

    // Initialize NVS
    NvsManager nvsManager;
    if (nvsManager.init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "❌ Failed to initialize NVS storage");
        return;
    }

    // Initialize ESP-NOW Mesh Coordinator BEFORE BLE
    ESPNowMeshCoordinator meshCoordinator;
    if (meshCoordinator.init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "❌ Failed to initialize ESP-NOW LED Mesh network");
        return;
    }
    
    // Enable adaptive mesh system for topology-aware routing
    ESP_LOGI(MAIN_TAG, "🔄 Enabling adaptive mesh system...");
    esp_err_t adaptive_result = meshCoordinator.enableAdaptiveMesh();
    if (adaptive_result == ESP_OK && meshCoordinator.isAdaptiveMeshEnabled()) {
        ESP_LOGI(MAIN_TAG, "✅ Adaptive mesh system enabled - ready for topology-aware networking");
        ESP_LOGI(MAIN_TAG, "📊 Adaptive mesh will begin neighbor discovery and topology analysis");
        ESP_LOGI(MAIN_TAG, "🔍 Watch for neighbor discovery beacons and topology updates in logs");
    } else {
        ESP_LOGE(MAIN_TAG, "❌ Failed to enable adaptive mesh system: %s", esp_err_to_name(adaptive_result));
        ESP_LOGE(MAIN_TAG, "⚠️ System cannot continue without adaptive mesh - aborting startup");
        return;
    }

    // Create and initialize the BLEGattServer object
    BLEGattServer bleGattServer;
    if (bleGattServer.init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "❌ Failed to initialize BLE GATT Server for mobile connection");
        return;
    }

    // Set device name
    if (bleGattServer.setDeviceName(DEVICE_NAME) != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "❌ Failed to set BLE device name to %s", DEVICE_NAME);
        return;
    }

    // Register GATT callbacks BEFORE adding profiles
    BLEGapHandler gapHandler;
    if (bleGattServer.registerGattCallbacks() != ESP_OK || gapHandler.registerGapCallbacks() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "❌ Failed to register BLE GATT/GAP callbacks");
        return;
    }

    // Create Gatt Profiles here. Do this for each profile
    // 1. PixelPacketProfile for LED control commands
    const std::string service_uuid_str = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
    const std::string characteristic_uuid_str = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
    std::shared_ptr<PixelPacketProfile> pixel_packet_profile = std::make_shared<PixelPacketProfile>(service_uuid_str, characteristic_uuid_str);
    bleGattServer.addProfile(pixel_packet_profile);
    
    // 2. NetworkHealthProfile for mesh analytics
    const std::string health_service_uuid_str = "12345678-1234-1234-1234-123456789abc";
    const std::string health_characteristic_uuid_str = "87654321-4321-4321-4321-cba987654321";
    std::shared_ptr<NetworkHealthProfile> health_profile = std::make_shared<NetworkHealthProfile>(health_service_uuid_str, health_characteristic_uuid_str);
    bleGattServer.addProfile(health_profile);
    
    // Create NetworkHealth monitor
    NetworkHealthMonitor networkHealthMonitor;
    health_profile->setNetworkHealthMonitor(&networkHealthMonitor);
    
    // Create LED controller
    LEDController ledController;
    if (ledController.begin() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "❌ Failed to initialize LED controller");
        return;
    }
    
    // Create LED packet processor
    LedPacketProcessor ledProcessor;
    ledProcessor.setLedControlCallback([&ledController](const GenericPacket& packet, const char* format_info) {
        ESP_LOGI(MAIN_TAG, "🌈 LED Pattern Received: %s (%zu bytes)", format_info, packet.getLength());
        
        // Process the packet with our LED controller
        esp_err_t ret = ledController.processPacket(packet);
        if (ret == ESP_OK) {
            ESP_LOGI(MAIN_TAG, "✅ LED pattern applied successfully");
        } else {
            ESP_LOGW(MAIN_TAG, "❌ Failed to apply LED pattern: %s", esp_err_to_name(ret));
        }
    });
    
    // Set up mesh coordinator callbacks - now generic
    meshCoordinator.setPacketCallback([&ledProcessor](const GenericPacket& packet) {
        ESP_LOGI(MAIN_TAG, "Received LED pattern from mesh: %zu bytes", packet.getLength());
        
        // Process LED pattern (handles current format and future formats automatically)
        if (!ledProcessor.processPacket(packet)) {
            ESP_LOGW(MAIN_TAG, "Failed to process LED pattern from mesh");
        }
    });
    
    meshCoordinator.setRoleChangeCallback([&meshCoordinator](NodeRole old_role, NodeRole new_role) {
        ESP_LOGI(MAIN_TAG, "LED Mesh role changed from %d to %d", (int)old_role, (int)new_role);
        if (new_role == NodeRole::MESH_ROOT_ACTIVE) {
            ESP_LOGI(MAIN_TAG, "Now MESH ROOT (BLE) - accepting BLE commands and distributing LED patterns");
        } else if (new_role == NodeRole::MESH_ROOT_AUTONOMOUS) {
            ESP_LOGI(MAIN_TAG, "Now MESH ROOT (Autonomous) - distributing LED patterns without BLE");
        } else {
            ESP_LOGI(MAIN_TAG, "Now MESH CLIENT - receiving LED patterns from root");
        }
    });

    // Set up BLE connection callbacks to notify mesh coordinator
    pixel_packet_profile->setBleConnectionCallback([&meshCoordinator, &pixel_packet_profile](bool connected) {
        if (connected) {
            // Check if BLE connection should be accepted (stabilization period)
            if (!meshCoordinator.shouldAcceptBleConnection()) {
                ESP_LOGW(MAIN_TAG, "🔒 BLE connection rejected - stabilization period active, forcing disconnect");
                pixel_packet_profile->forceDisconnect(); // Force disconnect to give clear feedback
                return; // Connection will be disconnected
            }
            
            ESP_LOGI(MAIN_TAG, "🔥 Mobile phone connected via BLE - becoming LED mesh root (Node 0x%04X)", 
                     meshCoordinator.getNodeId());
            meshCoordinator.onBleConnected();
            ESP_LOGI(MAIN_TAG, "🔥 BLE connection processing complete - new role: %s", 
                     meshCoordinator.getRoleString());
        } else {
            ESP_LOGI(MAIN_TAG, "🔥 Mobile phone disconnected - stepping down from root role (Node 0x%04X)", 
                     meshCoordinator.getNodeId());
            meshCoordinator.onBleDisconnected();
            ESP_LOGI(MAIN_TAG, "🔥 BLE disconnection processing complete - new role: %s", 
                     meshCoordinator.getRoleString());
        }
    });

    // Set up packet forwarding from BLE to mesh - now generic
    pixel_packet_profile->setPacketForwardCallback([&meshCoordinator, &ledProcessor](const GenericPacket& packet) {
        // Always process locally first (root node gets LED patterns too)
        ESP_LOGI(MAIN_TAG, "🔥 Processing LED pattern from mobile phone locally (%zu bytes)", packet.getLength());
        if (!ledProcessor.processPacket(packet)) {
            ESP_LOGW(MAIN_TAG, "🔥 Failed to process LED pattern locally");
        } else {
            ESP_LOGI(MAIN_TAG, "🔥 Successfully processed LED pattern locally");
        }
        
        // Forward to ESP-NOW mesh network if we're root
        if (meshCoordinator.isRootNode()) {
            ESP_LOGI(MAIN_TAG, "Broadcasting LED pattern to mesh network (%zu bytes)", packet.getLength());
            if (meshCoordinator.sendLEDPattern(packet) != ESP_OK) {
                ESP_LOGE(MAIN_TAG, "Failed to broadcast LED pattern to mesh");
            }
        } else {
            ESP_LOGW(MAIN_TAG, "Received LED pattern but not root - cannot broadcast to mesh");
        }
    });

    // Start ESP-NOW mesh network
    if (meshCoordinator.start() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "❌ Failed to start ESP-NOW LED mesh network");
        return;
    }

    // Allow BLE stack to settle before starting advertising (especially important on ESP32-C3)
    // Increased delay to reduce ESP-NOW interference with BLE connection establishment
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    bleGattServer.startAdvertising();

    // Network health monitoring is initialized above and connected to BLE profile
    
    ESP_LOGI(MAIN_TAG, "✅ ESP32 LED Mesh Node ready - Node ID: 0x%04X", meshCoordinator.getNodeId());
    ESP_LOGI(MAIN_TAG, "📱 Connect via BLE to '%s' to control LED patterns", DEVICE_NAME);

    while (true) {
        // Get current time for timing operations
        uint32_t now = esp_timer_get_time() / 1000;
        
        // LED updates are now handled by dedicated LED task with guaranteed timing
        // Main loop focuses on network coordination and system management
        
        // Check for autonomous root election
        meshCoordinator.checkForRootElection();
        
        // Check election timeout for advanced election system (optimized: every 100ms vs 15ms)
        static uint32_t lastElectionTimeoutCheck = 0;
        if (now - lastElectionTimeoutCheck >= 100) { // Check every 100ms instead of 15ms
            meshCoordinator.checkElectionTimeout();
            lastElectionTimeoutCheck = now;
        }
        
        // Root nodes send periodic announcements (every 5 seconds)
        // - BLE roots: Ensure autonomous roots know about superior BLE root and step down  
        // - Autonomous roots: Maintain root authority in absence of BLE root
        static uint32_t lastRootAnnouncement = 0;
        if (meshCoordinator.isRootNode() && (now - lastRootAnnouncement > 5000)) {
            meshCoordinator.sendRootAnnouncement();
            lastRootAnnouncement = now;
        }
        
        // Update adaptive mesh components for neighbor discovery and topology management
        static uint32_t lastAdaptiveMeshUpdate = 0;
        if (now - lastAdaptiveMeshUpdate >= 1000) { // Update every 1 second for responsive neighbor discovery
            static uint32_t updateCount = 0;
            if (++updateCount % 10 == 1) { // Log every 10th call (every 10 seconds) to avoid spam
                ESP_LOGI(MAIN_TAG, "🔄 Main loop calling updateAdaptiveMesh() (#%u)", updateCount);
            }
            meshCoordinator.updateAdaptiveMesh();
            lastAdaptiveMeshUpdate = now;
        }
        
        // Update and report network health every 60 seconds (reduced frequency)
        static uint32_t lastHealthUpdate = 0;
        if (now - lastHealthUpdate > 60000) {
            const auto& stats = meshCoordinator.getNetworkStats();
            size_t active_neighbors = meshCoordinator.getActiveNeighborCount();
            
            // Convert mesh stats to simple format and update network health
            MeshStats meshStats = {
                .packets_sent = stats.packets_sent,
                .packets_received = stats.packets_received,
                .packets_dropped = stats.packets_dropped,
                .send_failures = stats.send_failures
            };
            // TODO: Implement RSSI collection in mesh coordinator
            int8_t avg_rssi = -65;  // Placeholder RSSI value (good signal strength)
            
            networkHealthMonitor.updateMetrics(meshStats, active_neighbors, avg_rssi,
                                              meshCoordinator.isRootNode() ? (meshCoordinator.isBleConnected() ? 1 : 2) : 0,
                                              meshCoordinator.getReachableNodeCount());
            
            const NetworkHealth& health = networkHealthMonitor.getCurrentHealth();
            
            // Send network health via BLE characteristic for Flutter app
            health_profile->sendHealthUpdate();
            
            // Single concise status line with network health
            ESP_LOGI(MAIN_TAG, "Node 0x%04X | %s | LED:%s | Health:%u%% (%u neighbors, %u%% success)", 
                     meshCoordinator.getNodeId(), 
                     meshCoordinator.getRoleString(),
                     ledController.getCurrentSequenceType(),
                     health.overall_score,
                     health.active_neighbors,
                     health.packet_success_rate);
            
            // Show detailed health in logs for debugging
            const char* health_level = "UNKNOWN";
            switch (networkHealthMonitor.getHealthLevel()) {
                case NETWORK_EXCELLENT: health_level = "EXCELLENT"; break;
                case NETWORK_GOOD: health_level = "GOOD"; break; 
                case NETWORK_POOR: health_level = "POOR"; break;
                case NETWORK_CRITICAL: health_level = "CRITICAL"; break;
            }
            ESP_LOGD(MAIN_TAG, "📊 Network Health Details: Level=%s, RSSI=%ddBm, Uptime=%uh", 
                     health_level, health.avg_signal_strength, health.uptime_hours);
            
            lastHealthUpdate = now;
        }
        
        // Optional: Enable detailed mesh debugging (disabled in production)
        // Debug mesh neighbor discovery every 30 seconds
        static uint32_t lastNeighborDebug = 0;
        if (meshCoordinator.isAdaptiveMeshEnabled() && (now - lastNeighborDebug > 30000)) { // Every 30 seconds
            size_t neighborCount = meshCoordinator.getActiveNeighborCount();
            ESP_LOGI(MAIN_TAG, "🔍 NEIGHBOR DEBUG - Active neighbors: %zu, Adaptive mesh enabled: %s", 
                     neighborCount, meshCoordinator.isAdaptiveMeshEnabled() ? "YES" : "NO");
            
            // Print current network health data
            NetworkHealth health = networkHealthMonitor.getCurrentHealth();
            ESP_LOGI(MAIN_TAG, "📊 HEALTH DEBUG - Score: %d%%, Neighbors: %d, Success: %d%%, Role: %d",
                     health.overall_score, health.active_neighbors, health.packet_success_rate, health.mesh_role);
            
            lastNeighborDebug = now;
        }
        
        #ifdef CONFIG_LOG_MAXIMUM_LEVEL_DEBUG
        static uint32_t lastDetailedStatus = 0;
        if (meshCoordinator.isAdaptiveMeshEnabled() && (now - lastDetailedStatus > 300000)) { // Every 5 minutes
            meshCoordinator.printAdaptiveMeshStatus();
            lastDetailedStatus = now;
        }
        #endif
        
        vTaskDelay(pdMS_TO_TICKS(15)); // 15ms delay for smooth LED updates while reducing ESP-NOW/BLE interference
    }

}
