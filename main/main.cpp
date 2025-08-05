#include "bluetooth/ble_gatt_server.h"
#include "bluetooth/ble_gap_handler.h"
#include "bluetooth/gatt_profile.h"
#include "system/nvs_manager.h"
#include "bluetooth/pixel_packet_profile.h"
#include "mesh/espnow_mesh_coordinator.h"
#include "packet/led_packet_processor.h"
#include "packet/generic_packet.h"
#include "led/led_controller.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_log.h"

#include <functional>
#include <memory>

#define MAIN_TAG "ESP_LED_MESH"
#define DEVICE_NAME "ESP_LED_NODE"


extern "C" void app_main(void) {
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

    // Create Gatt Profiles here. Do this for each profile (In our case only PixelPacketProile for now)
    const std::string service_uuid_str = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
    const std::string characteristic_uuid_str = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
    std::shared_ptr<PixelPacketProfile> pixel_packet_profile = std::make_shared<PixelPacketProfile>(service_uuid_str, characteristic_uuid_str);
    bleGattServer.addProfile(pixel_packet_profile);
    
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
        if (ledController.processPacket(packet)) {
            ESP_LOGI(MAIN_TAG, "✅ LED pattern applied successfully");
        } else {
            ESP_LOGW(MAIN_TAG, "❌ Failed to apply LED pattern");
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
            ESP_LOGI(MAIN_TAG, "Now MESH ROOT - accepting BLE commands and distributing LED patterns");
        } else {
            ESP_LOGI(MAIN_TAG, "Now MESH CLIENT - receiving LED patterns from root");
        }
    });

    // Set up BLE connection callbacks to notify mesh coordinator
    pixel_packet_profile->setBleConnectionCallback([&meshCoordinator](bool connected) {
        if (connected) {
            ESP_LOGI(MAIN_TAG, "Mobile phone connected via BLE - becoming LED mesh root");
            meshCoordinator.onBleConnected();
        } else {
            ESP_LOGI(MAIN_TAG, "Mobile phone disconnected - stepping down from root role");
            meshCoordinator.onBleDisconnected();
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

    bleGattServer.startAdvertising();

    ESP_LOGI(MAIN_TAG, "✅ ESP32 LED Mesh Node ready - Node ID: 0x%04X", meshCoordinator.getNodeId());
    ESP_LOGI(MAIN_TAG, "📱 Connect via BLE to '%s' to control LED patterns", DEVICE_NAME);

    while (true) {
        // Get current time for timing operations
        uint32_t now = esp_timer_get_time() / 1000;
        
        // Update LED controller (must be called frequently for smooth animations)
        ledController.update();
        
        // Check for autonomous root election
        meshCoordinator.checkForRootElection();
        
        // If we're autonomous root, send periodic announcements (every 5 seconds)
        static uint32_t lastRootAnnouncement = 0;
        if (meshCoordinator.isAutonomousRoot() && (now - lastRootAnnouncement > 5000)) {
            meshCoordinator.sendRootAnnouncement();
            lastRootAnnouncement = now;
        }
        
        // Monitor system status and network health every 30 seconds
        static uint32_t lastStatusLog = 0;
        if (now - lastStatusLog > 30000) {
            const auto& stats = meshCoordinator.getNetworkStats();
            
            ESP_LOGI(MAIN_TAG, "LED Node 0x%04X - Role: %s - LED: %s", 
                     meshCoordinator.getNodeId(), 
                     meshCoordinator.getRoleString(),
                     ledController.getCurrentSequenceType());
            ESP_LOGI(MAIN_TAG, "Mesh Stats: Sent=%lu, Received=%lu, Dropped=%lu, Failures=%lu", 
                     stats.packets_sent, stats.packets_received, stats.packets_dropped, stats.send_failures);
            
            lastStatusLog = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay for smooth LED updates
    }

}
