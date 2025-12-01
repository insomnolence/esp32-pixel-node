#pragma once

#include "esp_err.h"

// Use real BT types when available, otherwise use stubs for compilation
#ifdef CONFIG_BT_ENABLED
#include "esp_bt_defs.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#else
#include "bluetooth/ble_types_stub.h"
#endif

#include <functional>

/**
 * @brief Abstract interface for Bluetooth Low Energy hardware interactions.
 */
class BleHardwareInterface {
public:
    virtual ~BleHardwareInterface() = default;

    // Controller & Stack Initialization
    // Handles both Controller and Bluedroid initialization/enabling with coexistence checks
    virtual esp_err_t initStack() = 0;

    // GAP
    virtual esp_err_t configAdvData(esp_ble_adv_data_t *adv_data) = 0;
    virtual esp_err_t startAdvertising(esp_ble_adv_params_t *adv_params) = 0;
    virtual esp_err_t setDeviceName(const char *name) = 0;

    // GATTS
    virtual esp_err_t registerGattsApp(uint16_t app_id) = 0;
    virtual esp_err_t createService(esp_gatt_if_t gatts_if, esp_gatt_srvc_id_t *service_id, uint16_t num_handle) = 0;
    virtual esp_err_t startService(uint16_t service_handle) = 0;
    virtual esp_err_t addChar(uint16_t service_handle, esp_bt_uuid_t *char_uuid, 
                              esp_gatt_perm_t perm, esp_gatt_char_prop_t property, 
                              esp_attr_value_t *char_val, esp_attr_control_t *control) = 0;
    virtual esp_err_t addCharDescr(uint16_t service_handle, esp_bt_uuid_t *descr_uuid,
                                   esp_gatt_perm_t perm, esp_attr_value_t *char_val,
                                   esp_attr_control_t *control) = 0;
    virtual esp_err_t sendResponse(esp_gatt_if_t gatts_if, uint16_t conn_id, uint32_t trans_id,
                                   esp_gatt_status_t status, esp_gatt_rsp_t *rsp) = 0;
    virtual esp_err_t sendIndication(esp_gatt_if_t gatts_if, uint16_t conn_id, uint16_t attr_handle,
                                     uint16_t length, uint8_t *value, bool need_confirm) = 0;
    virtual esp_err_t closeConnection(esp_gatt_if_t gatts_if, uint16_t conn_id) = 0;

    // Callbacks
    // The generic interface takes a C-style callback or a C++ handler?
    // ESP-IDF expects C function pointers. The wrapper implementation will handle that.
    // We need a way for the upper layer to receive events.
    
    using GattsCallback = void (*)(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    using GapCallback = void (*)(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

    virtual esp_err_t registerGattsCallback(GattsCallback callback) = 0;
    virtual esp_err_t registerGapCallback(GapCallback callback) = 0;
};
