#include "bluetooth/esp_idf_ble_hardware.h"
#include "esp_log.h"

static const char* TAG = "EspIdfBleHardware";

EspIdfBleHardware::EspIdfBleHardware() {}

esp_err_t EspIdfBleHardware::initStack() {
    esp_err_t ret;
    
    // Check if Bluedroid is already initialized (may happen with WiFi coexistence)
    esp_bt_controller_status_t bt_state = esp_bt_controller_get_status();
    if (bt_state == ESP_BT_CONTROLLER_STATUS_IDLE) {
        ESP_LOGI(TAG, "BT controller not initialized, initializing now");
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret) {
            ESP_LOGE(TAG, "%s bt controller init failed: %s", __func__, esp_err_to_name(ret));
            return ret;
        }
        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret) {
            ESP_LOGE(TAG, "%s bt controller enable failed: %s", __func__, esp_err_to_name(ret));
            return ret;
        }
    } else {
        ESP_LOGI(TAG, "BT controller already initialized (likely by WiFi coexistence)");
    }
    
    // Check if Bluedroid is already initialized
    esp_bluedroid_status_t bluedroid_status = esp_bluedroid_get_status();
    if (bluedroid_status == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        ret = esp_bluedroid_init();
        if (ret) {
            ESP_LOGE(TAG, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
            return ret;
        }
    } else {
        ESP_LOGI(TAG, "Bluedroid already initialized");
    }
    
    if (bluedroid_status != ESP_BLUEDROID_STATUS_ENABLED) {
        ret = esp_bluedroid_enable();
        if (ret) {
            ESP_LOGE(TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
            return ret;
        }
    } else {
        ESP_LOGI(TAG, "Bluedroid already enabled");
    }
    
    return ESP_OK;
}

esp_err_t EspIdfBleHardware::configAdvData(esp_ble_adv_data_t *adv_data) {
    return esp_ble_gap_config_adv_data(adv_data);
}

esp_err_t EspIdfBleHardware::startAdvertising(esp_ble_adv_params_t *adv_params) {
    return esp_ble_gap_start_advertising(adv_params);
}

esp_err_t EspIdfBleHardware::setDeviceName(const char *name) {
    return esp_ble_gap_set_device_name(name);
}

esp_err_t EspIdfBleHardware::registerGattsApp(uint16_t app_id) {
    return esp_ble_gatts_app_register(app_id);
}

esp_err_t EspIdfBleHardware::createService(esp_gatt_if_t gatts_if, esp_gatt_srvc_id_t *service_id, uint16_t num_handle) {
    return esp_ble_gatts_create_service(gatts_if, service_id, num_handle);
}

esp_err_t EspIdfBleHardware::startService(uint16_t service_handle) {
    return esp_ble_gatts_start_service(service_handle);
}

esp_err_t EspIdfBleHardware::addChar(uint16_t service_handle, esp_bt_uuid_t *char_uuid, 
                                     esp_gatt_perm_t perm, esp_gatt_char_prop_t property, 
                                     esp_attr_value_t *char_val, esp_attr_control_t *control) {
    return esp_ble_gatts_add_char(service_handle, char_uuid, perm, property, char_val, control);
}

esp_err_t EspIdfBleHardware::addCharDescr(uint16_t service_handle, esp_bt_uuid_t *descr_uuid,
                                          esp_gatt_perm_t perm, esp_attr_value_t *char_val,
                                          esp_attr_control_t *control) {
    return esp_ble_gatts_add_char_descr(service_handle, descr_uuid, perm, char_val, control);
}

esp_err_t EspIdfBleHardware::sendResponse(esp_gatt_if_t gatts_if, uint16_t conn_id, uint32_t trans_id,
                                          esp_gatt_status_t status, esp_gatt_rsp_t *rsp) {
    return esp_ble_gatts_send_response(gatts_if, conn_id, trans_id, status, rsp);
}

esp_err_t EspIdfBleHardware::sendIndication(esp_gatt_if_t gatts_if, uint16_t conn_id, uint16_t attr_handle,
                                            uint16_t length, uint8_t *value, bool need_confirm) {
    return esp_ble_gatts_send_indicate(gatts_if, conn_id, attr_handle, length, value, need_confirm);
}

esp_err_t EspIdfBleHardware::closeConnection(esp_gatt_if_t gatts_if, uint16_t conn_id) {
    return esp_ble_gatts_close(gatts_if, conn_id);
}

esp_err_t EspIdfBleHardware::registerGattsCallback(GattsCallback callback) {
    return esp_ble_gatts_register_callback(callback);
}

esp_err_t EspIdfBleHardware::registerGapCallback(GapCallback callback) {
    return esp_ble_gap_register_callback(callback);
}
