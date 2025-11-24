#include "system_control/global_objects.h"
#include "esp_heap_caps.h"

const char* GlobalObjects::TAG = "GlobalObjects";

// Static member definitions
HeapUtils::esp_unique_ptr<ESPNowMeshCoordinator> GlobalObjects::mesh_coordinator_;
HeapUtils::esp_unique_ptr<BLEGattServer> GlobalObjects::ble_server_;
HeapUtils::esp_unique_ptr<LEDController> GlobalObjects::led_controller_;
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
    // Track which critical components failed for detailed error reporting
    bool mesh_success = false;
    bool led_success = false;

    // Initialize MeshCoordinator (critical)
    mesh_success = initializeMeshCoordinator(status);
    if (!mesh_success) {
        ESP_LOGE(TAG, "CRITICAL: MeshCoordinator initialization failed - aborting");
        status.result = HeapUtils::InitStatus::Result::CRITICAL_FAILURE;
        cleanup();
        return status;
    }

    // Initialize LEDController (critical)
    led_success = initializeLEDController(status);
    if (!led_success) {
        ESP_LOGE(TAG, "CRITICAL: LEDController initialization failed - aborting");
        status.result = HeapUtils::InitStatus::Result::CRITICAL_FAILURE;
        cleanup();
        return status;
    }

    ESP_LOGI(TAG, "All critical components initialized successfully");

    // Initialize optional components (can fail gracefully)
    // Only proceed with optional components after critical components succeed
    bool optional_success = initializeBLEServer(status);
    if (!optional_success) {
        ESP_LOGW(TAG, "Optional component BLEServer failed - continuing with degraded functionality");
    }

    status.free_heap_after = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    initialized_ = true;

    // Determine final result
    if (optional_success) {
        status.result = HeapUtils::InitStatus::Result::SUCCESS;
        ESP_LOGI(TAG, "All global objects initialized successfully");
    } else {
        status.result = HeapUtils::InitStatus::Result::PARTIAL_SUCCESS;
        ESP_LOGW(TAG, "Global objects partially initialized (BLEServer failed)");
    }

    logMemoryUsage();
    status.logResults(TAG);

    return status;
}

bool GlobalObjects::initializeMeshCoordinator(HeapUtils::InitStatus& status) {
    ESP_LOGI(TAG, "Allocating ESPNowMeshCoordinator on heap...");

    auto result = HeapUtils::ObjectFactory<ESPNowMeshCoordinator>::create();

    if (result.success()) {
        mesh_coordinator_ = std::move(result.object);
        LOG_HEAP_ALLOCATION(TAG, "ESPNowMeshCoordinator", mesh_coordinator_.get());
        status.components_initialized++;
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to allocate ESPNowMeshCoordinator: %s",
                 result.result == HeapUtils::ObjectFactory<ESPNowMeshCoordinator>::CreateResult::OUT_OF_MEMORY ?
                 "OUT_OF_MEMORY" : "INITIALIZATION_FAILED");
        status.components_failed++;
        return false;
    }
}

bool GlobalObjects::initializeBLEServer(HeapUtils::InitStatus& status) {
    ESP_LOGI(TAG, "Allocating BLEGattServer on heap...");

    auto result = HeapUtils::ObjectFactory<BLEGattServer>::create();

    if (result.success()) {
        ble_server_ = std::move(result.object);
        LOG_HEAP_ALLOCATION(TAG, "BLEGattServer", ble_server_.get());
        status.components_initialized++;
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to allocate BLEGattServer: %s",
                 result.result == HeapUtils::ObjectFactory<BLEGattServer>::CreateResult::OUT_OF_MEMORY ?
                 "OUT_OF_MEMORY" : "INITIALIZATION_FAILED");
        status.components_failed++;
        return false;
    }
}

bool GlobalObjects::initializeLEDController(HeapUtils::InitStatus& status) {
    ESP_LOGI(TAG, "Allocating LEDController on heap...");

    auto result = HeapUtils::ObjectFactory<LEDController>::create();

    if (result.success()) {
        led_controller_ = std::move(result.object);
        LOG_HEAP_ALLOCATION(TAG, "LEDController", led_controller_.get());
        status.components_initialized++;
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to allocate LEDController: %s",
                 result.result == HeapUtils::ObjectFactory<LEDController>::CreateResult::OUT_OF_MEMORY ?
                 "OUT_OF_MEMORY" : "INITIALIZATION_FAILED");
        status.components_failed++;
        return false;
    }
}

void GlobalObjects::cleanup() {
    ESP_LOGI(TAG, "Cleaning up global objects...");

    // Reset in reverse order of initialization
    led_controller_.reset();
    ble_server_.reset();
    mesh_coordinator_.reset();

    initialized_ = false;
    ESP_LOGI(TAG, "Global objects cleanup complete");
}

bool GlobalObjects::hasValidObjects() {
    return initialized_ && 
           mesh_coordinator_ && 
           led_controller_;
    // Note: BLE server is optional, so not checked here
}

ESPNowMeshCoordinator& GlobalObjects::getMeshCoordinator() {
    if (!mesh_coordinator_) {
        ESP_LOGE(TAG, "FATAL: MeshCoordinator accessed before initialization - restarting");
        esp_restart();
    }
    return *mesh_coordinator_;
}

BLEGattServer& GlobalObjects::getBLEServer() {
    if (!ble_server_) {
        ESP_LOGE(TAG, "FATAL: BLEServer accessed before initialization - restarting");
        esp_restart();
    }
    return *ble_server_;
}

LEDController& GlobalObjects::getLEDController() {
    if (!led_controller_) {
        ESP_LOGE(TAG, "FATAL: LEDController accessed before initialization - restarting");
        esp_restart();
    }
    return *led_controller_;
}

void GlobalObjects::logMemoryUsage() {
    uint32_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    uint32_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);

    ESP_LOGI(TAG, "HEAP ALLOCATION SUMMARY:");
    ESP_LOGI(TAG, "  MeshCoordinator: %s (~800 bytes estimated)",
             mesh_coordinator_ ? "Allocated" : "Failed");
    ESP_LOGI(TAG, "  BLEGattServer: %s (~400 bytes estimated)",
             ble_server_ ? "Allocated" : "Failed");
    ESP_LOGI(TAG, "  LEDController: %s (~300 bytes estimated)",
             led_controller_ ? "Allocated" : "Failed");
    ESP_LOGI(TAG, "Current heap: %lu bytes free, minimum seen: %lu bytes", free_heap, min_free);
}
