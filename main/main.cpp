// main.cpp
#include "ble_gatt_server.h"
#include "ble_gap_handler.h"
#include "gatt_profile.h"
#include "nvs_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_log.h"

#include <functional>

#define GATTS_TAG "GATTS_DEMO"
#define TEST_DEVICE_NAME "GATTS_DEMO"


extern "C" void app_main(void) {
    // Initialize NVS
    NvsManager nvsManager;
    if (nvsManager.init() != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to initialize NVS");
        return;
    }

    //Initialize the GATT profile.
    //gl_profile_tab[PROFILE_A_APP_ID].gatts_cb = gatts_profile_a_event_handler;
    //gl_profile_tab[PROFILE_A_APP_ID].gatts_if = ESP_GATT_IF_NONE;


    // Create and initialize the BLEGattServer object
    BLEGattServer bleGattServer;
    if (bleGattServer.init() != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to initialize BLE GATT Server");
        return;
    }

    GattProfile gattProfile;
    // Set the profile event handler
    bleGattServer.setProfileEventHandler(std::bind(&GattProfile::gattsProfileAEventHandler, &gattProfile, 
       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), PROFILE_A_APP_ID); // Change here

    // Register GATT and GAP callbacks
    BLEGapHandler gapHandler;
    if (bleGattServer.registerGattCallbacks() != ESP_OK || gapHandler.registerGapCallbacks() != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to register GATT or GAP callbacks");
        return;
    }

    // Register GATT application
    if (bleGattServer.registerGattApp(PROFILE_A_APP_ID) != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to register GATT application");
        return;
    }




    // Set device name
    if (bleGattServer.setDeviceName(TEST_DEVICE_NAME) != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to set device name");
        return;
    }

    // Start advertising
    gattProfile.startAdvertising();

    ESP_LOGI(GATTS_TAG, "Application initialized");

    // Add an infinite loop here
    while (true) {
        // Small delay here to prevent a watchdog timeout
        // and allow other tasks to run.
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second (1000 milliseconds)
    }

}
