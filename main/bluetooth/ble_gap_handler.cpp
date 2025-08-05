#include "ble_gap_handler.h"
#include "esp_log.h"

const char* BLEGapHandler::TAG = "BLEGapHandler";

BLEGapHandler::BLEGapHandler() {
}

BLEGapHandler::~BLEGapHandler() {
}

esp_err_t BLEGapHandler::registerGapCallbacks() {
    esp_err_t ret = esp_ble_gap_register_callback(BLEGapHandler::gapEventHandler);
    if (ret) {
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return ret;
    }
    return ESP_OK;
}

void BLEGapHandler::gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    ESP_LOGI(TAG, "GAP event %d", event);
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising data set complete");
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan response data set complete");
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising stop failed");
            } else {
                ESP_LOGI(TAG, "Stop adv successfully");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d, latency = %d, timeout = %d",
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
