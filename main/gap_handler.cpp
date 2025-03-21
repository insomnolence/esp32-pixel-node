#include "gap_handler.h"
#include "gap_gatt_data.h"
#include "esp_log.h"

#define GATTS_TAG "GATTS_DEMO"

esp_err_t register_gap_callbacks() {
    esp_err_t ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return ret;
    }
    return ESP_OK;
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    ESP_LOGI(GATTS_TAG, "GAP event %d", event);
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                if (esp_ble_gap_start_advertising(&adv_params) != ESP_OK) {
                    ESP_LOGE(GATTS_TAG, "Failed to start advertising");
                }
                else {
                    ESP_LOGI(GATTS_TAG, "Advertising started");
                }
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
                if (esp_ble_gap_start_advertising(&adv_params) != ESP_OK) {
                    ESP_LOGE(GATTS_TAG, "Failed to start advertising");
                }
                else {
                    ESP_LOGI(GATTS_TAG, "Advertising started");
                }
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising start failed");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising stop failed");
            } else {
                ESP_LOGI(GATTS_TAG, "Stop adv successfully");
                adv_config_done = 0;
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d, latency = %d, timeout = %d",
                     param->update_conn_params.status,
                     param->update_conn_params.min_int,
                     param->update_conn_params.max_int,
                     param->update_conn_params.conn_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}