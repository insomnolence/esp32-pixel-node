#ifndef BLE_GATT_SERVER_H_
#define BLE_GATT_SERVER_H_

#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

esp_err_t ble_controller_init_and_enable();
esp_err_t ble_bluedroid_init_and_enable();
esp_err_t register_gatt_callbacks();
esp_err_t register_gatt_app(uint16_t app_id);
esp_err_t set_device_name(const char *device_name);

#endif // BLE_GATT_SERVER_H_

