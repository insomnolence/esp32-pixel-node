#pragma once

#include "esp_err.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "bluetooth/gatt_profile.h" // Include for gatts_profile_inst_t
#include "bluetooth/ble_hardware_interface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <functional>
#include <vector>
#include <memory>

class BLEGattServer {
public:
    // Constructor with hardware interface for dependency injection (preferred for testability)
    explicit BLEGattServer(BleHardwareInterface* hardware);
    ~BLEGattServer();

    esp_err_t init();
    esp_err_t registerGattCallbacks();
    esp_err_t registerGattApp(uint16_t app_id);
    esp_err_t startAdvertising();
    esp_err_t setDeviceName(const char *device_name);
    void setProfileEventHandler(std::function<void(esp_gatts_cb_event_t, esp_gatt_if_t, esp_ble_gatts_cb_param_t*)> eventHandler, uint8_t profile_id); // Change here
    static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    void addProfile(std::shared_ptr<GattProfile> profile);
    void setAdvertisingCallback(std::function<void()> callback);
    void validateGattServiceHealth(); // Debug function for GATT service validation
    static void staticValidateGattHealth(); // Static version callable from anywhere

private:
    BleHardwareInterface* hardware_; // Hardware abstraction layer
    std::vector<std::shared_ptr<GattProfile>> profile_list;
    void handleGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    static const char* TAG;
    esp_ble_adv_params_t adv_params;
    static BLEGattServer* instance; // Declare the static instance variable
    static SemaphoreHandle_t instance_mutex; // Mutex for thread-safe singleton access
    static uint32_t callback_counter; // Debug counter for callback invocations
    std::function<void()> advertisingCallback;
};
