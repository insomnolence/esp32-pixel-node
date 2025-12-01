#pragma once

#include "bluetooth/ble_hardware_interface.h"

#ifdef CONFIG_BT_ENABLED
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#endif

class EspIdfBleHardware : public BleHardwareInterface {
public:
    EspIdfBleHardware();
    virtual ~EspIdfBleHardware() = default;

    esp_err_t initStack() override;

    esp_err_t configAdvData(esp_ble_adv_data_t *adv_data) override;
    esp_err_t startAdvertising(esp_ble_adv_params_t *adv_params) override;
    esp_err_t setDeviceName(const char *name) override;

    esp_err_t registerGattsApp(uint16_t app_id) override;
    esp_err_t createService(esp_gatt_if_t gatts_if, esp_gatt_srvc_id_t *service_id, uint16_t num_handle) override;
    esp_err_t startService(uint16_t service_handle) override;
    esp_err_t addChar(uint16_t service_handle, esp_bt_uuid_t *char_uuid, 
                      esp_gatt_perm_t perm, esp_gatt_char_prop_t property, 
                      esp_attr_value_t *char_val, esp_attr_control_t *control) override;
    esp_err_t addCharDescr(uint16_t service_handle, esp_bt_uuid_t *descr_uuid,
                           esp_gatt_perm_t perm, esp_attr_value_t *char_val,
                           esp_attr_control_t *control) override;
    esp_err_t sendResponse(esp_gatt_if_t gatts_if, uint16_t conn_id, uint32_t trans_id,
                           esp_gatt_status_t status, esp_gatt_rsp_t *rsp) override;
    esp_err_t sendIndication(esp_gatt_if_t gatts_if, uint16_t conn_id, uint16_t attr_handle,
                             uint16_t length, uint8_t *value, bool need_confirm) override;
    esp_err_t closeConnection(esp_gatt_if_t gatts_if, uint16_t conn_id) override;

    esp_err_t registerGattsCallback(GattsCallback callback) override;
    esp_err_t registerGapCallback(GapCallback callback) override;
};
