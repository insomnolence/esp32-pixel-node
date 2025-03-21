// main.cpp
#include "ble_gatt_server.h"
#include "gap_handler.h"
#include "gatt_profile.h"
#include "nvs_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_log.h"

#define GATTS_TAG "GATTS_DEMO"
#define TEST_DEVICE_NAME "GATTS_DEMO"


extern "C" void app_main(void) {
    // Initialize NVS
    if (nvs_manager_init() != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to initialize NVS");
        return;
    }

    //Initialize the GATT profile.
    gl_profile_tab[PROFILE_A_APP_ID].gatts_cb = gatts_profile_a_event_handler;
    gl_profile_tab[PROFILE_A_APP_ID].gatts_if = ESP_GATT_IF_NONE;

    // Initialize and enable the BT controller
    if (ble_controller_init_and_enable() != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to initialize BT controller");
        return;
    }

    // Initialize and enable Bluedroid
    if (ble_bluedroid_init_and_enable() != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to initialize Bluedroid");
        return;
    }

    // Register GATT and GAP callbacks
    if (register_gatt_callbacks() != ESP_OK || register_gap_callbacks() != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to register GATT or GAP callbacks");
        return;
    }

    // Register GATT application
    if (register_gatt_app(PROFILE_A_APP_ID) != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "Failed to register GATT application");
        return;
    }

    ESP_LOGI(GATTS_TAG, "Application initialized");

    // Add an infinite loop here
    while (true) {
        // Small delay here to prevent a watchdog timeout
        // and allow other tasks to run.
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second (1000 milliseconds)
    }

}