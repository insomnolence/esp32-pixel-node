#ifndef GAP_GATT_DATA_H_
#define GAP_GATT_DATA_H_

#include "esp_gap_ble_api.h"
#include <stdint.h>

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

// Static and const for data that does not change
static uint8_t adv_config_done = 0;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#endif // GAP_GATT_DATA_H_