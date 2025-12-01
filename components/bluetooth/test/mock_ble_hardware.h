#pragma once

#include "bluetooth/ble_hardware_interface.h"
#include <cstring>
#include <string>
#include <vector>

class MockBleHardware : public BleHardwareInterface {
public:
    MockBleHardware() {}
    virtual ~MockBleHardware() = default;

    // State recording
    bool initStackCalled = false;
    bool configAdvDataCalled = false;
    bool startAdvertisingCalled = false;
    std::string deviceName;
    std::vector<uint16_t> registeredApps;
    
    GattsCallback gattsCb = nullptr;
    GapCallback gapCb = nullptr;

    // Interface Implementation
    esp_err_t initStack() override {
        initStackCalled = true;
        return ESP_OK;
    }

    esp_err_t configAdvData(esp_ble_adv_data_t *adv_data) override {
        configAdvDataCalled = true;
        return ESP_OK;
    }

    esp_err_t startAdvertising(esp_ble_adv_params_t *adv_params) override {
        startAdvertisingCalled = true;
        return ESP_OK;
    }

    esp_err_t setDeviceName(const char *name) override {
        deviceName = name;
        return ESP_OK;
    }

    esp_err_t registerGattsApp(uint16_t app_id) override {
        registeredApps.push_back(app_id);
        // Simulate immediate registration event if callback is set?
        // Typically async, so test should trigger it manually.
        return ESP_OK;
    }

    esp_err_t createService(esp_gatt_if_t gatts_if, esp_gatt_srvc_id_t *service_id, uint16_t num_handle) override {
        return ESP_OK;
    }

    esp_err_t startService(uint16_t service_handle) override {
        return ESP_OK;
    }

    esp_err_t addChar(uint16_t service_handle, esp_bt_uuid_t *char_uuid, 
                      esp_gatt_perm_t perm, esp_gatt_char_prop_t property, 
                      esp_attr_value_t *char_val, esp_attr_control_t *control) override {
        return ESP_OK;
    }

    esp_err_t addCharDescr(uint16_t service_handle, esp_bt_uuid_t *descr_uuid,
                           esp_gatt_perm_t perm, esp_attr_value_t *char_val,
                           esp_attr_control_t *control) override {
        return ESP_OK;
    }

    esp_err_t sendResponse(esp_gatt_if_t gatts_if, uint16_t conn_id, uint32_t trans_id,
                           esp_gatt_status_t status, esp_gatt_rsp_t *rsp) override {
        return ESP_OK;
    }

    esp_err_t sendIndication(esp_gatt_if_t gatts_if, uint16_t conn_id, uint16_t attr_handle,
                             uint16_t length, uint8_t *value, bool need_confirm) override {
        return ESP_OK;
    }

    esp_err_t closeConnection(esp_gatt_if_t gatts_if, uint16_t conn_id) override {
        return ESP_OK;
    }

    esp_err_t registerGattsCallback(GattsCallback callback) override {
        gattsCb = callback;
        return ESP_OK;
    }

    esp_err_t registerGapCallback(GapCallback callback) override {
        gapCb = callback;
        return ESP_OK;
    }

    // Helper to simulate events
    void simulateGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
        if (gattsCb) {
            gattsCb(event, gatts_if, param);
        }
    }
};
