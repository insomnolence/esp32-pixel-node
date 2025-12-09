#include "bluetooth/ble_gatt_server.h"
#include "bluetooth/ble_gap_handler.h"
#include "bluetooth/gatt_profile.h"
#include "system_control/nvs_manager.h"
#include "bluetooth/pixel_packet_profile.h"
#include "bluetooth/network_health_profile.h"
#include "mesh/espnow_mesh_coordinator.h"
#include "packet/led_packet_processor.h"
#include "packet/generic_packet.h"
#include "led/led_controller.h"

#ifdef CONFIG_BUTTON_INTERFACE_ENABLED
#include "system_control/button_manager.h"
#include "system_control/button_logic.h"
#include "system_control/button_feedback.h"
#include "common/button_feedback_types.h"
#include "system_control/dual_button_detector.h"
#include "system_control/root_takeover_manager.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"

// Stack optimization utilities
#include "system_control/global_objects.h"
#include "system_control/stack_monitor.h"

#include <functional>
#include <memory>

#include "driver/gpio.h"

#define MAIN_TAG "ESP_LED_MESH"
#define DEVICE_NAME "ESP_LED_NODE"
#define CONFIG_INDICATOR_IO GPIO_NUM_6 // GPIO for indicator LED

extern "C" void app_main(void) {
    // 🔄 RESET REASON: Log why we rebooted (critical for battery debugging)
    esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGW(MAIN_TAG, "Reset reason: %d (%s)", reset_reason,
        reset_reason == ESP_RST_POWERON ? "POWER_ON - normal boot" :
        reset_reason == ESP_RST_SW ? "SOFTWARE - esp_restart() called" :
        reset_reason == ESP_RST_PANIC ? "PANIC - software crash" :
        reset_reason == ESP_RST_INT_WDT ? "INT_WDT - interrupt watchdog" :
        reset_reason == ESP_RST_TASK_WDT ? "TASK_WDT - task watchdog" :
        reset_reason == ESP_RST_WDT ? "WDT - other watchdog" :
        reset_reason == ESP_RST_DEEPSLEEP ? "DEEPSLEEP - woke from sleep" :
        reset_reason == ESP_RST_BROWNOUT ? "BROWNOUT - voltage dropped below 2.51V" :
        reset_reason == ESP_RST_USB ? "USB - reset by USB peripheral (normal)" :
        reset_reason == ESP_RST_JTAG ? "JTAG - reset by JTAG" :
        reset_reason == ESP_RST_PWR_GLITCH ? "PWR_GLITCH - power glitch detected" :
        reset_reason == ESP_RST_CPU_LOCKUP ? "CPU_LOCKUP - double exception" : "UNKNOWN");

    // 📊 STACK OPTIMIZATION: Monitor initial stack usage
    ESP_LOGI(MAIN_TAG, "🚀 Starting ESP32 LED Mesh with heap-optimized architecture");
    LOG_CURRENT_STACK();
    
    // Setup Indicator LED on IO6
    gpio_set_direction(CONFIG_INDICATOR_IO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_INDICATOR_IO, 1); // Turn on indicator LED
    
    
    // Enable debug logging for mesh coordinator
    esp_log_level_set("ESPNowMeshCoordinator", ESP_LOG_DEBUG);
    ESP_LOGI(MAIN_TAG, "🔍 Enabled debug logging for mesh coordinator components");
    
    // 🏭 HEAP ALLOCATION: Initialize large objects on heap for stack safety
    auto init_status = GlobalObjects::initialize();
    if (!init_status.canContinue()) {
        ESP_LOGE(MAIN_TAG, "❌ Critical failure during global object initialization");
        ESP_LOGE(MAIN_TAG, "💀 System cannot continue - restarting...");
        esp_restart();
    }
    
    // 💡 Initialize LED controller (now on heap)
    auto& ledController = GlobalObjects::getLEDController();
    if (ledController.begin() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "❌ Failed to initialize LED controller");
        return;
    }

    ledController.update(); //Blank LEDs initially
    // 📊 Check stack usage after heap allocation
    LOG_CURRENT_STACK();

    // Initialize NVS (small object, keep on stack)
    NvsManager nvsManager;
    if (nvsManager.init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "❌ Failed to initialize NVS storage");
        return;
    }
    
    // 🔗 Initialize ESP-NOW Mesh Coordinator (now on heap)
    auto& meshCoordinator = GlobalObjects::getMeshCoordinator();
    if (meshCoordinator.init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "❌ Failed to initialize ESP-NOW LED Mesh network");
        return;
    }
    

    // 📱 Initialize BLE GATT Server (now on heap)
    auto& bleGattServer = GlobalObjects::getBLEServer();
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
    BLEGapHandler gapHandler(&GlobalObjects::getBleHardware()); // Small object, keep on stack
    if (bleGattServer.registerGattCallbacks() != ESP_OK || gapHandler.registerGapCallbacks() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "❌ Failed to register BLE GATT/GAP callbacks");
        return;
    }

    // Create Gatt Profiles here. Do this for each profile
    // 1. PixelPacketProfile for LED control commands (UUIDs from Kconfig)
    const std::string service_uuid_str = CONFIG_BLE_SERVICE_UUID;
    const std::string characteristic_uuid_str = CONFIG_BLE_CHARACTERISTIC_UUID;
    std::shared_ptr<PixelPacketProfile> pixel_packet_profile = std::make_shared<PixelPacketProfile>(service_uuid_str, characteristic_uuid_str);
    bleGattServer.addProfile(pixel_packet_profile);

    // 2. NetworkHealthProfile for mesh analytics (UUIDs from Kconfig)
    const std::string health_service_uuid_str = CONFIG_BLE_HEALTH_SERVICE_UUID;
    const std::string health_characteristic_uuid_str = CONFIG_BLE_HEALTH_CHARACTERISTIC_UUID;
    std::shared_ptr<NetworkHealthProfile> health_profile = std::make_shared<NetworkHealthProfile>(health_service_uuid_str, health_characteristic_uuid_str);
    bleGattServer.addProfile(health_profile);

    // Connect NetworkHealthMonitor to the profile
    auto& healthMonitor = GlobalObjects::getNetworkHealthMonitor();
    health_profile->setNetworkHealthMonitor(&healthMonitor);
    health_profile->startPeriodicUpdates(5000); // Update every 5 seconds

    // Set up pattern broadcast callback for button-triggered modes
    ledController.setPatternBroadcastCallback([&meshCoordinator](const GenericPacket& packet) {
        if (meshCoordinator.isRootNode()) {
            ESP_LOGI(MAIN_TAG, "Broadcasting button pattern to mesh network (%zu bytes)", packet.getLength());
            if (meshCoordinator.sendLEDPattern(packet) != ESP_OK) {
                ESP_LOGW(MAIN_TAG, "Failed to broadcast button pattern to mesh");
            }
        }
    });

    // Set up root status callback for return-to-idle when root disappears (root election redesign)
    ledController.setRootStatusCallback([&meshCoordinator]() -> bool {
        return meshCoordinator.hasActiveRoot() || meshCoordinator.isRootNode();
    });

    // Check stack usage after core system initialization
    LOG_CURRENT_STACK();

#ifdef CONFIG_BUTTON_INTERFACE_ENABLED
    // Initialize Button Interface (ESP32C3 custom board)
    ESP_LOGI(MAIN_TAG, "🔘 Initializing button interface for standalone control...");
    
    // Get ButtonManager from GlobalObjects (already initialized)
    auto& buttonManager = GlobalObjects::getButtonManager();

    // Other components still static locals for now (TODO: Move to GlobalObjects)
    static ButtonFeedbackController buttonFeedback(&ledController);
    static ButtonLogic buttonLogic(&ledController, &meshCoordinator, &buttonFeedback);
    static DualButtonDetector dualButtonDetector;
    static RootTakeoverManager rootTakeoverManager;
    
    // Initialize button feedback controller
    if (buttonFeedback.init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "⚠️ Failed to initialize button feedback controller");
    }
    
    // Initialize button logic system
    if (buttonLogic.init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "⚠️ Failed to initialize button logic");
    }
    
    // Set up button event handling through ButtonLogic architecture
    // Note: Static locals don't need to be captured
    buttonManager.setEventCallback([](ButtonManager::ButtonEvent event) {
        // First, pass all button events to dual-button detector for root takeover detection
        dualButtonDetector.processButtonEvent(event);
        
        // Then pass to ButtonLogic for business logic processing
        buttonLogic.handleButtonEvent(event);
    });
    
    // Initialize dual-button detection and root takeover after mesh coordinator is ready
    if (dualButtonDetector.init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "⚠️ Failed to initialize dual-button detector");
    }
    
    if (rootTakeoverManager.init(&meshCoordinator) != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "⚠️ Failed to initialize root takeover manager");
    }
    
    // Set up dual-button callbacks
    dualButtonDetector.setDualButtonCallback([](DualButtonDetector::DualButtonEvent event) {
        switch (event) {
            case DualButtonDetector::DUAL_PRESS_DETECTED:
                ESP_LOGI(MAIN_TAG, "🎯 DUAL BUTTON PRESS DETECTED - Preparing root takeover...");
                break;
            case DualButtonDetector::DUAL_HOLD_CONFIRMED:
                ESP_LOGI(MAIN_TAG, "🚀 DUAL BUTTON HOLD CONFIRMED - Starting root takeover!");
                rootTakeoverManager.startTakeover();
                break;
            case DualButtonDetector::DUAL_PRESS_CANCELLED:
                ESP_LOGI(MAIN_TAG, "❌ Dual button press cancelled - buttons released");
                rootTakeoverManager.cancelTakeover();
                break;
        }
    });
    
    rootTakeoverManager.setTakeoverCallback([](RootTakeoverManager::TakeoverResult result) {
        switch (result) {
            case RootTakeoverManager::SUCCESS:
                ESP_LOGI(MAIN_TAG, "✅ ROOT TAKEOVER SUCCESS - Now controlling mesh network!");
                break;
            case RootTakeoverManager::FAILED_BLE_PRIORITY:
                ESP_LOGI(MAIN_TAG, "📱 Root takeover blocked - Mobile app has priority");
                break;
            case RootTakeoverManager::FAILED_NETWORK_ERROR:
                ESP_LOGE(MAIN_TAG, "❌ Root takeover failed - Network error");
                break;
            case RootTakeoverManager::FAILED_TIMEOUT:
                ESP_LOGE(MAIN_TAG, "❌ Root takeover failed - Timeout");
                break;
            case RootTakeoverManager::CANCELLED_USER:
                ESP_LOGI(MAIN_TAG, "🛑 Root takeover cancelled by user");
                break;
        }
    });
    
    // Set up visual feedback callback for LED feedback during takeover
    rootTakeoverManager.setVisualFeedbackCallback([](RootTakeoverManager::TakeoverState state) {
        ESP_LOGD(MAIN_TAG, "Visual feedback for takeover state: %d", (int)state);
        
        // Use ButtonFeedbackController for local-only feedback
        switch (state) {
            case RootTakeoverManager::DUAL_PRESS_DETECTED:
                buttonFeedback.showFeedback(DUAL_PRESS_DETECTED);
                break;
            case RootTakeoverManager::INITIATING_TAKEOVER:
            case RootTakeoverManager::WAITING_NETWORK_SYNC:
                buttonFeedback.showFeedback(TAKEOVER_IN_PROGRESS);
                break;
            case RootTakeoverManager::TAKEOVER_COMPLETE:
                buttonFeedback.showFeedback(TAKEOVER_SUCCESS);
                break;
            case RootTakeoverManager::TAKEOVER_FAILED:
            case RootTakeoverManager::BLE_OVERRIDE_ACTIVE:
                buttonFeedback.showFeedback(TAKEOVER_FAILED);
                break;
            default:
                // No feedback for other states
                break;
        }
    });
    
    ESP_LOGI(MAIN_TAG, "✅ Button interface ready - GPIO %d (sequence), GPIO %d (random)", 
            CONFIG_BUTTON_1_GPIO, CONFIG_BUTTON_2_GPIO);
    ESP_LOGI(MAIN_TAG, "🎯 Dual-button root takeover enabled - Hold both buttons 2s for mesh control");

#else
    ESP_LOGI(MAIN_TAG, "Button interface disabled - BLE-only control mode");
#endif
    
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
    meshCoordinator.setPacketCallback([&ledProcessor, &ledController, &meshCoordinator](const GenericPacket& packet) {
        // Skip processing if we're the autonomous root (button-controlled) - we generate patterns, not receive them
        if (meshCoordinator.getCurrentRole() == NodeRole::MESH_ROOT_AUTONOMOUS) {
            ESP_LOGD(MAIN_TAG, "Ignoring LED pattern - autonomous root generates patterns, doesn't receive them");
            return;
        }

        ESP_LOGI(MAIN_TAG, "Received LED pattern from mesh: %zu bytes", packet.getLength());

        // Track pattern reception time for return-to-idle behavior (root election redesign)
        ledController.onPatternReceived();

        // Process LED pattern (handles current format and future formats automatically)
        if (!ledProcessor.processPacket(packet)) {
            ESP_LOGW(MAIN_TAG, "Failed to process LED pattern from mesh");
        }
    });

    meshCoordinator.setRoleChangeCallback([&meshCoordinator
#ifdef CONFIG_BUTTON_INTERFACE_ENABLED
                                          , &buttonLogic, &rootTakeoverManager
#endif
                                          ](NodeRole old_role, NodeRole new_role) {
#ifdef CONFIG_BUTTON_INTERFACE_ENABLED
        // Notify button logic and root takeover manager of role changes
        bool is_root = (new_role == NodeRole::MESH_ROOT_ACTIVE || new_role == NodeRole::MESH_ROOT_AUTONOMOUS);
        buttonLogic.onMeshRoleChanged(is_root);
        rootTakeoverManager.onMeshRoleChanged(old_role, new_role);
#endif
        ESP_LOGI(MAIN_TAG, "LED Mesh role changed from %d to %d", (int)old_role, (int)new_role);
        if (new_role == NodeRole::MESH_ROOT_ACTIVE) {
            ESP_LOGI(MAIN_TAG, "Now MESH ROOT (BLE) - accepting BLE commands and distributing LED patterns");
        } else if (new_role == NodeRole::MESH_ROOT_AUTONOMOUS) {
            ESP_LOGI(MAIN_TAG, "Now MESH ROOT (Autonomous) - distributing LED patterns without BLE");
            // Autonomous root: buttons remain active, no state reset needed
        } else {
            ESP_LOGI(MAIN_TAG, "Now MESH CLIENT - receiving LED patterns from root");
#ifdef CONFIG_BUTTON_INTERFACE_ENABLED
            // Client mode: button logic handles state reset automatically
            ESP_LOGI(MAIN_TAG, "🔴 Button 1 state reset to IDLE due to client role");
#endif
        }
    });

    // Set up BLE connection callbacks to notify mesh coordinator
    pixel_packet_profile->setBleConnectionCallback([&meshCoordinator, &pixel_packet_profile
#ifdef CONFIG_BUTTON_INTERFACE_ENABLED
                                                   , &buttonLogic, &rootTakeoverManager
#endif
                                                   ](bool connected) {
#ifdef CONFIG_BUTTON_INTERFACE_ENABLED
        // Notify button logic and root takeover manager of BLE connection events
        buttonLogic.onBleConnectionChanged(connected);
        if (connected) {
            rootTakeoverManager.onBleConnected();
        }
#endif
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
            ESP_LOGI(MAIN_TAG, "Mobile phone disconnected - stepping down from root role (Node 0x%04X)",
                     meshCoordinator.getNodeId());
            meshCoordinator.onBleDisconnected();
            ESP_LOGI(MAIN_TAG, "BLE disconnection processing complete - new role: %s",
                     meshCoordinator.getRoleString());
        }
    });

    // Set up BLE displacement callback for dual-BLE scenario (root election redesign)
    meshCoordinator.setBleDisplacementCallback([&pixel_packet_profile]() {
        ESP_LOGW(MAIN_TAG, "BLE displacement triggered - sending notification and disconnecting");
        // Send notification to phone before disconnecting
        pixel_packet_profile->sendRootTransferredNotification();
        vTaskDelay(pdMS_TO_TICKS(100)); // Brief delay for notification to send
        pixel_packet_profile->forceDisconnect();
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
    
    // Log stack usage for ESP32-C3 debugging
    TaskHandle_t main_task = xTaskGetCurrentTaskHandle();
    UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(main_task);
    ESP_LOGI(MAIN_TAG, "📊 Main task stack usage: %u bytes free of %u total", 
             stack_high_water * sizeof(StackType_t), CONFIG_ESP_MAIN_TASK_STACK_SIZE);
    
    ESP_LOGI(MAIN_TAG, "✅ ESP32 LED Mesh Node ready - Node ID: 0x%04X", meshCoordinator.getNodeId());
    ESP_LOGI(MAIN_TAG, "📱 Connect via BLE to '%s' to control LED patterns", DEVICE_NAME);

    while (true) {
        // Get current time for timing operations
        uint32_t now = esp_timer_get_time() / 1000;
        
        // LED updates are now handled by dedicated LED task with guaranteed timing
        // Main loop focuses on network coordination and system management
        
        // 📊 STACK MONITORING: Periodic check for stack health (every 30 seconds)
        StackMonitor::periodicCheck(30000);
        
        // Check for autonomous root election
        meshCoordinator.checkForRootElection();
        
#ifdef CONFIG_BUTTON_INTERFACE_ENABLED
        // Process button events and dual-button detection
        buttonManager.processEvents();
        buttonLogic.update();
        buttonFeedback.update();
        dualButtonDetector.update();
        rootTakeoverManager.update();
#endif

        // Root nodes send periodic announcements (every 5 seconds)
        // - BLE roots: Ensure autonomous roots know about superior BLE root and step down
        // - Autonomous roots: Maintain root authority in absence of BLE root

        // Update and report network health every 10 seconds
        static uint32_t lastHealthUpdate = 0;
        if (now - lastHealthUpdate > 10000) {
            const auto& stats = meshCoordinator.getNetworkStats();
            size_t active_neighbors = meshCoordinator.getActiveNeighborCount();
            int8_t avg_rssi = meshCoordinator.getAverageNeighborRSSI();

            // Convert NodeRole to uint8_t for NetworkHealthMonitor
            uint8_t role = 0;
            if (meshCoordinator.getCurrentRole() == NodeRole::MESH_ROOT_ACTIVE) {
                role = 1; // BLE root
            } else if (meshCoordinator.getCurrentRole() == NodeRole::MESH_ROOT_AUTONOMOUS) {
                role = 2; // Autonomous root
            }

            // Update NetworkHealthMonitor with mesh stats
            MeshStats mesh_stats;
            mesh_stats.packets_sent = stats.packets_sent;
            mesh_stats.packets_received = stats.packets_received;
            mesh_stats.packets_dropped = stats.packets_dropped;
            mesh_stats.send_failures = stats.send_failures;

            // Calculate total nodes: use root's count if available, otherwise 1 + neighbors
            uint8_t total_nodes = (uint8_t)stats.total_nodes;
            if (total_nodes == 0) {
                // Fallback: at minimum we have ourselves + our visible neighbors
                total_nodes = 1 + (uint8_t)active_neighbors;
            }

            healthMonitor.updateMetrics(mesh_stats,
                                       (uint8_t)active_neighbors,
                                       avg_rssi,
                                       role,
                                       total_nodes);

            // Log network health status
            ESP_LOGI(MAIN_TAG, "Node 0x%04X | %s | LED:%s | PktRcvd:%lu, PktSent:%lu, Failures:%lu, Nbrs:%zu",
                     meshCoordinator.getNodeId(),
                     meshCoordinator.getRoleString(),
                     ledController.getCurrentSequenceType(),
                     stats.packets_received,
                     stats.packets_sent,
                     stats.send_failures,
                     active_neighbors);

            lastHealthUpdate = now;
        }
        
        // ESP32-C3 Stack Safety Monitoring (every 60 seconds)
        static uint32_t lastStackMonitor = 0;
        if (now - lastStackMonitor > 60000) { // Check every 60 seconds
            UBaseType_t stack_free = uxTaskGetStackHighWaterMark(NULL);
            uint32_t stack_free_bytes = stack_free * sizeof(StackType_t);
            
            if (stack_free_bytes < 1024) { // Less than 1KB free - CRITICAL
                ESP_LOGE(MAIN_TAG, "🚨 STACK CRITICAL: Only %u bytes free of %u total", 
                         stack_free_bytes, CONFIG_ESP_MAIN_TASK_STACK_SIZE);
            } else if (stack_free_bytes < 2048) { // Less than 2KB free - WARNING  
                ESP_LOGW(MAIN_TAG, "⚠️ STACK WARNING: %u bytes free of %u total", 
                         stack_free_bytes, CONFIG_ESP_MAIN_TASK_STACK_SIZE);
            } else {
                ESP_LOGI(MAIN_TAG, "📊 Stack health: %u bytes free of %u total", 
                         stack_free_bytes, CONFIG_ESP_MAIN_TASK_STACK_SIZE);
            }
            lastStackMonitor = now;
        }

        vTaskDelay(pdMS_TO_TICKS(15)); // 15ms delay for smooth LED updates while reducing ESP-NOW/BLE interference
    }

    gpio_set_level(CONFIG_INDICATOR_IO, 0); // Turn off indicator LED

}
