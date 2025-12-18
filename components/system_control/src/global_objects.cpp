#include "system_control/global_objects.h"
#include "esp_heap_caps.h"
#include "sdkconfig.h"

const char* GlobalObjects::TAG = "GlobalObjects";

// Static member definitions
HeapUtils::esp_unique_ptr<EspIdfMeshHardware> GlobalObjects::mesh_hardware_;
HeapUtils::esp_unique_ptr<EspIdfSystemHardware> GlobalObjects::system_hardware_;
#ifdef CONFIG_BT_ENABLED
HeapUtils::esp_unique_ptr<EspIdfBleHardware> GlobalObjects::ble_hardware_;
HeapUtils::esp_unique_ptr<BLEGattServer> GlobalObjects::ble_server_;
#endif
HeapUtils::esp_unique_ptr<ESPNowMeshCoordinator> GlobalObjects::mesh_coordinator_;
HeapUtils::esp_unique_ptr<LEDStrip> GlobalObjects::led_strip_;
HeapUtils::esp_unique_ptr<LEDController> GlobalObjects::led_controller_;
HeapUtils::esp_unique_ptr<ButtonManager> GlobalObjects::button_manager_;
HeapUtils::esp_unique_ptr<NetworkHealthMonitor> GlobalObjects::network_health_monitor_;
bool GlobalObjects::initialized_ = false;

HeapUtils::InitStatus GlobalObjects::initialize() {
    HeapUtils::InitStatus status;
    status.free_heap_before = heap_caps_get_free_size(MALLOC_CAP_8BIT);

    ESP_LOGI(TAG, "Initializing global objects on heap...");
    ESP_LOGI(TAG, "Free heap before allocation: %lu bytes", status.free_heap_before);

    if (initialized_) {
        ESP_LOGW(TAG, "Global objects already initialized");
        status.result = HeapUtils::InitStatus::Result::SUCCESS;
        return status;
    }

    // Initialize critical components (must succeed)
    bool system_hardware_success = false;
    bool hardware_success = false;
    bool mesh_success = false;
    bool strip_success = false;
    bool led_success = false;
    bool health_monitor_success = false;
    #ifdef CONFIG_BUTTON_INTERFACE_ENABLED
    bool button_manager_success = false;
    #endif

    // 1. System Hardware
    system_hardware_success = initializeSystemHardware(status);
    if (!system_hardware_success) {
        ESP_LOGE(TAG, "CRITICAL: SystemHardware initialization failed");
        status.result = HeapUtils::InitStatus::Result::CRITICAL_FAILURE;
        cleanup();
        return status;
    }

    // 2. Mesh Hardware
    hardware_success = initializeMeshHardware(status);
    if (!hardware_success) {
        ESP_LOGE(TAG, "CRITICAL: MeshHardware initialization failed");
        status.result = HeapUtils::InitStatus::Result::CRITICAL_FAILURE;
        cleanup();
        return status;
    }

    // 3. BLE Hardware (only when BT is enabled)
#ifdef CONFIG_BT_ENABLED
    bool ble_hardware_success = initializeBleHardware(status);
    if (!ble_hardware_success) {
        ESP_LOGW(TAG, "BleHardware initialization failed - BLE will be disabled");
    }
#else
    bool ble_hardware_success = false;
    ESP_LOGI(TAG, "BLE disabled at compile time (CONFIG_BT_ENABLED=n)");
#endif

    // 4. Mesh Coordinator
    mesh_success = initializeMeshCoordinator(status, mesh_hardware_.get());
    if (!mesh_success) {
        ESP_LOGE(TAG, "CRITICAL: MeshCoordinator initialization failed");
        status.result = HeapUtils::InitStatus::Result::CRITICAL_FAILURE;
        cleanup();
        return status;
    }

    // 5. LED Strip
    strip_success = initializeLEDStrip(status);
    if (!strip_success) {
        ESP_LOGE(TAG, "CRITICAL: LEDStrip initialization failed");
        status.result = HeapUtils::InitStatus::Result::CRITICAL_FAILURE;
        cleanup();
        return status;
    }

    // 6. LED Controller
    led_success = initializeLEDController(status, led_strip_.get());
    if (!led_success) {
        ESP_LOGE(TAG, "CRITICAL: LEDController initialization failed");
        status.result = HeapUtils::InitStatus::Result::CRITICAL_FAILURE;
        cleanup();
        return status;
    }

    // 7. Button Manager
    #ifdef CONFIG_BUTTON_INTERFACE_ENABLED
    button_manager_success = initializeButtonManager(status, system_hardware_.get());
    if (!button_manager_success) {
        ESP_LOGE(TAG, "CRITICAL: ButtonManager initialization failed");
        status.result = HeapUtils::InitStatus::Result::CRITICAL_FAILURE;
        cleanup();
        return status;
    }
    #endif

    // 8. Network Health Monitor
    health_monitor_success = initializeNetworkHealthMonitor(status);
    if (!health_monitor_success) {
        ESP_LOGE(TAG, "CRITICAL: NetworkHealthMonitor initialization failed");
        status.result = HeapUtils::InitStatus::Result::CRITICAL_FAILURE;
        cleanup();
        return status;
    }

    ESP_LOGI(TAG, "All critical components initialized successfully");

    // 9. BLE Server (Optional, only when BT is enabled)
    bool optional_success = false;
#ifdef CONFIG_BT_ENABLED
    if (ble_hardware_success) {
        optional_success = initializeBLEServer(status, ble_hardware_.get());
        if (!optional_success) {
            ESP_LOGW(TAG, "Optional component BLEServer failed");
        }
    }
#endif

    status.free_heap_after = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    initialized_ = true;

    if (ble_hardware_success && optional_success) {
        status.result = HeapUtils::InitStatus::Result::SUCCESS;
    } else {
        status.result = HeapUtils::InitStatus::Result::PARTIAL_SUCCESS;
    }

    logMemoryUsage();
    status.logResults(TAG);

    return status;
}

bool GlobalObjects::initializeSystemHardware(HeapUtils::InitStatus& status) {
    ESP_LOGI(TAG, "Allocating EspIdfSystemHardware...");
    auto result = HeapUtils::ObjectFactory<EspIdfSystemHardware>::create();
    if (result.success()) {
        system_hardware_ = std::move(result.object);
        LOG_HEAP_ALLOCATION(TAG, "EspIdfSystemHardware", system_hardware_.get());
        status.components_initialized++;
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to allocate EspIdfSystemHardware");
        status.components_failed++;
        return false;
    }
}

bool GlobalObjects::initializeMeshHardware(HeapUtils::InitStatus& status) {
    ESP_LOGI(TAG, "Allocating EspIdfMeshHardware...");
    auto result = HeapUtils::ObjectFactory<EspIdfMeshHardware>::create();
    if (result.success()) {
        mesh_hardware_ = std::move(result.object);
        LOG_HEAP_ALLOCATION(TAG, "EspIdfMeshHardware", mesh_hardware_.get());
        status.components_initialized++;
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to allocate EspIdfMeshHardware");
        status.components_failed++;
        return false;
    }
}

#ifdef CONFIG_BT_ENABLED
bool GlobalObjects::initializeBleHardware(HeapUtils::InitStatus& status) {
    ESP_LOGI(TAG, "Allocating EspIdfBleHardware...");
    auto result = HeapUtils::ObjectFactory<EspIdfBleHardware>::create();
    if (result.success()) {
        ble_hardware_ = std::move(result.object);
        LOG_HEAP_ALLOCATION(TAG, "EspIdfBleHardware", ble_hardware_.get());
        status.components_initialized++;
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to allocate EspIdfBleHardware");
        status.components_failed++;
        return false;
    }
}
#endif

bool GlobalObjects::initializeMeshCoordinator(HeapUtils::InitStatus& status, MeshHardwareInterface* hw) {
    ESP_LOGI(TAG, "Allocating ESPNowMeshCoordinator...");
    auto result = HeapUtils::ObjectFactory<ESPNowMeshCoordinator>::create(hw);
    if (result.success()) {
        mesh_coordinator_ = std::move(result.object);
        LOG_HEAP_ALLOCATION(TAG, "ESPNowMeshCoordinator", mesh_coordinator_.get());
        status.components_initialized++;
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to allocate ESPNowMeshCoordinator");
        status.components_failed++;
        return false;
    }
}

bool GlobalObjects::initializeLEDStrip(HeapUtils::InitStatus& status) {
    ESP_LOGI(TAG, "Allocating LEDStrip...");
    auto result = HeapUtils::ObjectFactory<LEDStrip>::create(DEFAULT_LED_COUNT, DEFAULT_LED_PIN);
    if (result.success()) {
        led_strip_ = std::move(result.object);
        LOG_HEAP_ALLOCATION(TAG, "LEDStrip", led_strip_.get());
        status.components_initialized++;
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to allocate LEDStrip");
        status.components_failed++;
        return false;
    }
}

bool GlobalObjects::initializeLEDController(HeapUtils::InitStatus& status, ILedStrip* strip) {
    ESP_LOGI(TAG, "Allocating LEDController...");
    auto result = HeapUtils::ObjectFactory<LEDController>::create(strip);
    if (result.success()) {
        led_controller_ = std::move(result.object);
        LOG_HEAP_ALLOCATION(TAG, "LEDController", led_controller_.get());
        status.components_initialized++;
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to allocate LEDController");
        status.components_failed++;
        return false;
    }
}

#ifdef CONFIG_BUTTON_INTERFACE_ENABLED
bool GlobalObjects::initializeButtonManager(HeapUtils::InitStatus& status, SystemHardwareInterface* hw) {
    ESP_LOGI(TAG, "Allocating ButtonManager...");
    auto result = HeapUtils::ObjectFactory<ButtonManager>::create(hw);
    if (result.success()) {
        button_manager_ = std::move(result.object);
        if (button_manager_->init(CONFIG_BUTTON_1_GPIO, CONFIG_BUTTON_2_GPIO, CONFIG_BUTTON_DEBOUNCE_MS) != ESP_OK) {
             ESP_LOGE(TAG, "ButtonManager init failed");
             return false;
        }
        LOG_HEAP_ALLOCATION(TAG, "ButtonManager", button_manager_.get());
        status.components_initialized++;
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to allocate ButtonManager");
        status.components_failed++;
        return false;
    }
}
#endif

bool GlobalObjects::initializeNetworkHealthMonitor(HeapUtils::InitStatus& status) {
    ESP_LOGI(TAG, "Allocating NetworkHealthMonitor...");
    auto result = HeapUtils::ObjectFactory<NetworkHealthMonitor>::create();
    if (result.success()) {
        network_health_monitor_ = std::move(result.object);
        LOG_HEAP_ALLOCATION(TAG, "NetworkHealthMonitor", network_health_monitor_.get());
        status.components_initialized++;
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to allocate NetworkHealthMonitor");
        status.components_failed++;
        return false;
    }
}

#ifdef CONFIG_BT_ENABLED
bool GlobalObjects::initializeBLEServer(HeapUtils::InitStatus& status, BleHardwareInterface* hw) {
    ESP_LOGI(TAG, "Allocating BLEGattServer...");
    auto result = HeapUtils::ObjectFactory<BLEGattServer>::create(hw);
    if (result.success()) {
        ble_server_ = std::move(result.object);
        LOG_HEAP_ALLOCATION(TAG, "BLEGattServer", ble_server_.get());
        status.components_initialized++;
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to allocate BLEGattServer");
        status.components_failed++;
        return false;
    }
}
#endif

void GlobalObjects::cleanup() {
    ESP_LOGI(TAG, "Cleaning up global objects...");

    led_controller_.reset();
    led_strip_.reset();
#ifdef CONFIG_BT_ENABLED
    ble_server_.reset();
#endif
    mesh_coordinator_.reset();
    network_health_monitor_.reset();

#ifdef CONFIG_BT_ENABLED
    ble_hardware_.reset();
#endif
    mesh_hardware_.reset();
    button_manager_.reset();
    system_hardware_.reset();

    initialized_ = false;
    ESP_LOGI(TAG, "Global objects cleanup complete");
}

bool GlobalObjects::hasValidObjects() {
    return initialized_ && 
           mesh_hardware_ &&
           system_hardware_ &&
           mesh_coordinator_ &&
           led_strip_ && 
           led_controller_;
}

EspIdfSystemHardware& GlobalObjects::getSystemHardware() {
    if (!system_hardware_) {
        ESP_LOGE(TAG, "FATAL: SystemHardware accessed before initialization");
        esp_restart();
    }
    return *system_hardware_;
}

EspIdfMeshHardware& GlobalObjects::getMeshHardware() {
    if (!mesh_hardware_) {
        ESP_LOGE(TAG, "FATAL: MeshHardware accessed before initialization");
        esp_restart();
    }
    return *mesh_hardware_;
}

#ifdef CONFIG_BT_ENABLED
EspIdfBleHardware& GlobalObjects::getBleHardware() {
    if (!ble_hardware_) {
        ESP_LOGE(TAG, "FATAL: BleHardware accessed before initialization");
        esp_restart();
    }
    return *ble_hardware_;
}
#endif

ESPNowMeshCoordinator& GlobalObjects::getMeshCoordinator() {
    if (!mesh_coordinator_) {
        ESP_LOGE(TAG, "FATAL: MeshCoordinator accessed before initialization");
        esp_restart();
    }
    return *mesh_coordinator_;
}

#ifdef CONFIG_BT_ENABLED
BLEGattServer& GlobalObjects::getBLEServer() {
    if (!ble_server_) {
        ESP_LOGE(TAG, "FATAL: BLEServer accessed before initialization");
        esp_restart();
    }
    return *ble_server_;
}
#endif

LEDStrip& GlobalObjects::getLEDStrip() {
    if (!led_strip_) {
        ESP_LOGE(TAG, "FATAL: LEDStrip accessed before initialization");
        esp_restart();
    }
    return *led_strip_;
}

LEDController& GlobalObjects::getLEDController() {
    if (!led_controller_) {
        ESP_LOGE(TAG, "FATAL: LEDController accessed before initialization");
        esp_restart();
    }
    return *led_controller_;
}

ButtonManager& GlobalObjects::getButtonManager() {
    if (!button_manager_) {
        ESP_LOGE(TAG, "FATAL: ButtonManager accessed before initialization");
        esp_restart();
    }
    return *button_manager_;
}

NetworkHealthMonitor& GlobalObjects::getNetworkHealthMonitor() {
    if (!network_health_monitor_) {
        ESP_LOGE(TAG, "FATAL: NetworkHealthMonitor accessed before initialization");
        esp_restart();
    }
    return *network_health_monitor_;
}

void GlobalObjects::logMemoryUsage() {
    uint32_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    uint32_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);

    ESP_LOGI(TAG, "HEAP ALLOCATION SUMMARY:");
    ESP_LOGI(TAG, "  MeshHardware: %s", mesh_hardware_ ? "Allocated" : "Failed");
    ESP_LOGI(TAG, "  SystemHardware: %s", system_hardware_ ? "Allocated" : "Failed");
#ifdef CONFIG_BT_ENABLED
    ESP_LOGI(TAG, "  BleHardware: %s", ble_hardware_ ? "Allocated" : "Failed");
    ESP_LOGI(TAG, "  BLEGattServer: %s", ble_server_ ? "Allocated" : "Failed");
#else
    ESP_LOGI(TAG, "  BleHardware: Disabled (CONFIG_BT_ENABLED=n)");
    ESP_LOGI(TAG, "  BLEGattServer: Disabled (CONFIG_BT_ENABLED=n)");
#endif
    ESP_LOGI(TAG, "  MeshCoordinator: %s", mesh_coordinator_ ? "Allocated" : "Failed");
    ESP_LOGI(TAG, "  LEDStrip: %s", led_strip_ ? "Allocated" : "Failed");
    ESP_LOGI(TAG, "  LEDController: %s", led_controller_ ? "Allocated" : "Failed");
    ESP_LOGI(TAG, "  ButtonManager: %s", button_manager_ ? "Allocated" : "Failed");
    ESP_LOGI(TAG, "  NetworkHealthMonitor: %s", network_health_monitor_ ? "Allocated" : "Failed");
    ESP_LOGI(TAG, "Current heap: %lu bytes free, minimum seen: %lu bytes", free_heap, min_free);
}