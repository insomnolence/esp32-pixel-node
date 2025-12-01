#pragma once

#include "system_control/heap_manager.h"
#include "mesh/espnow_mesh_coordinator.h"
#include "mesh/esp_idf_mesh_hardware.h"
#include "system_control/esp_idf_system_hardware.h"
#ifdef CONFIG_BT_ENABLED
#include "bluetooth/ble_gatt_server.h"
#include "bluetooth/esp_idf_ble_hardware.h"
#endif
#include "led/led_strip.h"
#include "led/led_controller.h" // Added
#include "system_control/button_manager.h" // Added
#include "system/network_health.h" // Added

/**
 * Global object manager for safe heap allocation of large service objects
 * Provides singleton access with proper initialization error handling
 */
class GlobalObjects {
private:
    static const char* TAG;
    static HeapUtils::esp_unique_ptr<EspIdfMeshHardware> mesh_hardware_;
    static HeapUtils::esp_unique_ptr<EspIdfSystemHardware> system_hardware_;
#ifdef CONFIG_BT_ENABLED
    static HeapUtils::esp_unique_ptr<EspIdfBleHardware> ble_hardware_;
    static HeapUtils::esp_unique_ptr<BLEGattServer> ble_server_;
#endif
    static HeapUtils::esp_unique_ptr<ESPNowMeshCoordinator> mesh_coordinator_;
    static HeapUtils::esp_unique_ptr<LEDStrip> led_strip_;
    static HeapUtils::esp_unique_ptr<LEDController> led_controller_;
    static HeapUtils::esp_unique_ptr<ButtonManager> button_manager_;
    static HeapUtils::esp_unique_ptr<NetworkHealthMonitor> network_health_monitor_;
    static bool initialized_;

public:
    static HeapUtils::InitStatus initialize();
    static void cleanup();

    // Safe accessors
    static EspIdfMeshHardware& getMeshHardware();
    static EspIdfSystemHardware& getSystemHardware();
#ifdef CONFIG_BT_ENABLED
    static EspIdfBleHardware& getBleHardware();
    static BLEGattServer& getBLEServer();
#endif
    static ESPNowMeshCoordinator& getMeshCoordinator();
    static LEDStrip& getLEDStrip();
    static LEDController& getLEDController();
    static ButtonManager& getButtonManager();
    static NetworkHealthMonitor& getNetworkHealthMonitor();

    // Status checks
    static bool isInitialized() { return initialized_; }
    static bool hasValidObjects();

    // Memory diagnostics
    static void logMemoryUsage();

private:
    static bool initializeMeshHardware(HeapUtils::InitStatus& status);
    static bool initializeSystemHardware(HeapUtils::InitStatus& status);
#ifdef CONFIG_BT_ENABLED
    static bool initializeBleHardware(HeapUtils::InitStatus& status);
    static bool initializeBLEServer(HeapUtils::InitStatus& status, BleHardwareInterface* hw);
#endif
    static bool initializeMeshCoordinator(HeapUtils::InitStatus& status, MeshHardwareInterface* hw);
    static bool initializeLEDStrip(HeapUtils::InitStatus& status);
    static bool initializeLEDController(HeapUtils::InitStatus& status, ILedStrip* strip);
    static bool initializeButtonManager(HeapUtils::InitStatus& status, SystemHardwareInterface* hw);
    static bool initializeNetworkHealthMonitor(HeapUtils::InitStatus& status);
};
