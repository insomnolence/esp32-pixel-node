#include "ble_gatt_server.h"
#include "ble_gap_handler.h"
#include "gatt_profile.h"
#include "nvs_manager.h"
#include "pixel_packet_profile.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_log.h"

#include <functional>
#include <memory>

#define GATTS_TAG "GATTS_DEMO"
#define TEST_DEVICE_NAME "GATTS_DEMO"


extern "C" void app_main(void) {
    // Initialize NVS
    NvsManager nvsManager;
    if (nvsManager.init() != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to initialize NVS");
        return;
    }

    // Create and initialize the BLEGattServer object
    BLEGattServer bleGattServer;
    if (bleGattServer.init() != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to initialize BLE GATT Server");
        return;
    }

    // Set device name
    if (bleGattServer.setDeviceName(TEST_DEVICE_NAME) != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to set device name");
        return;
    }

    // Create Gatt Profiles here. Do this for each profile (In our case only PixelPacketProile for now)
    const std::string service_uuid_str = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
    const std::string characteristic_uuid_str = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
    std::shared_ptr<PixelPacketProfile> pixel_packet_profile = std::make_shared<PixelPacketProfile>(service_uuid_str, characteristic_uuid_str);
    bleGattServer.addProfile(pixel_packet_profile);
    
    // Register GATT and GAP callbacks
    BLEGapHandler gapHandler;
    if (bleGattServer.registerGattCallbacks() != ESP_OK || gapHandler.registerGapCallbacks() != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to register GATT or GAP callbacks");
        return;
    }

    bleGattServer.startAdvertising();

    ESP_LOGI(GATTS_TAG, "Application initialized");

    while (true) {
        // Small delay here to prevent a watchdog timeout
        // and allow other tasks to run.
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second (1000 milliseconds)
    }

}
