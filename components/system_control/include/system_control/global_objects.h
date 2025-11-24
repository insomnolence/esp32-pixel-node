#pragma once

#include "system_control/heap_manager.h"
#include "mesh/espnow_mesh_coordinator.h"
#include "bluetooth/ble_gatt_server.h"
#include "led/led_controller.h"
#include "esp_log.h"

/**
 * Global object manager for safe heap allocation of large service objects
 * Provides singleton access with proper initialization error handling
 */
class GlobalObjects {
private:
    static const char* TAG;
    
    static HeapUtils::esp_unique_ptr<ESPNowMeshCoordinator> mesh_coordinator_;
    static HeapUtils::esp_unique_ptr<BLEGattServer> ble_server_;
    static HeapUtils::esp_unique_ptr<LEDController> led_controller_;
    static bool initialized_;
    
public:
    static HeapUtils::InitStatus initialize();
    static void cleanup();
    
    // Safe accessors - return reference, restart system if not initialized
    // These objects are preconditions for system operation, not optional
    static ESPNowMeshCoordinator& getMeshCoordinator();
    static BLEGattServer& getBLEServer();
    static LEDController& getLEDController();
    
    // Status checks
    static bool isInitialized() { return initialized_; }
    static bool hasValidObjects();
    
    // Memory diagnostics
    static void logMemoryUsage();
    
private:
    static bool initializeMeshCoordinator(HeapUtils::InitStatus& status);
    static bool initializeBLEServer(HeapUtils::InitStatus& status);
    static bool initializeLEDController(HeapUtils::InitStatus& status);
};
