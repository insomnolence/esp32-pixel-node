#ifndef GATT_PROFILE_H_
#define GATT_PROFILE_H_

#include "esp_err.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "gap_gatt_data.h"
#include "esp_gap_ble_api.h"

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

class GattProfile {
public:
    GattProfile();
    ~GattProfile();

    void gattsProfileAEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    void startAdvertising();
    void setServiceHandle(uint16_t service_handle);
    uint16_t getServiceHandle();

private:
    static const char* TAG;
    uint16_t service_handle;
    esp_gatt_srvc_id_t profile_service_id;
    esp_bt_uuid_t my_service_uuid;
    esp_bt_uuid_t my_characteristic_uuid;
    uint16_t char_handle;
    uint16_t descr_handle;
    esp_gatt_char_prop_t a_property;
    esp_ble_adv_data_t adv_data;
    esp_ble_adv_data_t scan_rsp_data;
    static esp_bt_uuid_t convertStringToUuid(const char *uuid_str);
};

extern GattProfile gattProfile;

#endif // GATT_PROFILE_H_
