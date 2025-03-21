#ifndef GATT_PROFILE_H_
#define GATT_PROFILE_H_

#include "esp_err.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"

//  Consider using a typedef to make the struct definition more readable
typedef struct gatts_profile_inst_t {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    uint16_t descr_handle;
} gatts_profile_inst_t;

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
extern gatts_profile_inst_t gl_profile_tab[PROFILE_NUM];

void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#endif // GATT_PROFILE_H_
