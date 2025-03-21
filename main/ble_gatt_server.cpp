#include "ble_gatt_server.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "gatts_event_handler.h"

#define GATTS_TAG "GATTS_DEMO"

esp_err_t ble_controller_init_and_enable() {
    esp_err_t ret;
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t ble_bluedroid_init_and_enable() {
    esp_err_t ret;
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t register_gatt_callbacks() {
    esp_err_t ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return ret;
    }
    return ESP_OK;
}

esp_err_t register_gatt_app(uint16_t app_id) {
    esp_err_t ret = esp_ble_gatts_app_register(app_id);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return ret;
    }
    return ESP_OK;
}

esp_err_t set_device_name(const char *device_name){
    esp_err_t ret = esp_ble_gap_set_device_name(device_name);
      if (ret)
    {
        ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", ret);
        return ret;
    }
    return ESP_OK;
}
