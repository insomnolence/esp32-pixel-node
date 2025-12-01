#pragma once

// Stub type definitions for when CONFIG_BT_ENABLED is false
// These allow the bluetooth component interfaces to compile without the real BT stack

#include <stdint.h>
#include <stdbool.h>

// From esp_bt_defs.h
#define ESP_UUID_LEN_16 2
#define ESP_UUID_LEN_32 4
#define ESP_UUID_LEN_128 16

typedef struct {
    uint16_t len;
    union {
        uint16_t uuid16;
        uint32_t uuid32;
        uint8_t uuid128[ESP_UUID_LEN_128];
    } uuid;
} esp_bt_uuid_t;

// From esp_gatt_defs.h
typedef uint8_t esp_gatt_if_t;
typedef uint8_t esp_gatt_status_t;
typedef uint8_t esp_gatt_perm_t;
typedef uint8_t esp_gatt_char_prop_t;

#define ESP_GATT_IF_NONE 0xFF
#define ESP_GATT_OK 0
#define ESP_GATT_NOT_FOUND 0x05
#define ESP_GATT_WRITE_NOT_PERMIT 0x03
#define ESP_GATT_PERM_READ 0x01
#define ESP_GATT_PERM_WRITE 0x10
#define ESP_GATT_CHAR_PROP_BIT_READ 0x02
#define ESP_GATT_CHAR_PROP_BIT_WRITE_NR 0x04
#define ESP_GATT_CHAR_PROP_BIT_NOTIFY 0x10
#define ESP_GATT_CHAR_PROP_BIT_INDICATE 0x20
#define ESP_GATT_UUID_CHAR_CLIENT_CONFIG 0x2902

typedef struct {
    bool is_primary;
    struct {
        esp_bt_uuid_t uuid;
        uint8_t inst_id;
    } id;
} esp_gatt_srvc_id_t;

typedef struct {
    uint16_t attr_max_len;
    uint16_t attr_len;
    uint8_t *attr_value;
} esp_attr_value_t;

typedef struct {
    uint8_t auto_rsp;
} esp_attr_control_t;

typedef struct {
    uint16_t handle;
    uint16_t offset;
    uint16_t len;
    uint8_t value[600];
    uint8_t auth_req;
} esp_gatt_value_t;

typedef struct {
    esp_gatt_value_t attr_value;
} esp_gatt_rsp_t;

// From esp_gatts_api.h
typedef enum {
    ESP_GATTS_REG_EVT = 0,
    ESP_GATTS_READ_EVT = 1,
    ESP_GATTS_WRITE_EVT = 2,
    ESP_GATTS_EXEC_WRITE_EVT = 3,
    ESP_GATTS_MTU_EVT = 4,
    ESP_GATTS_CONF_EVT = 5,
    ESP_GATTS_UNREG_EVT = 6,
    ESP_GATTS_CREATE_EVT = 7,
    ESP_GATTS_ADD_INCL_SRVC_EVT = 8,
    ESP_GATTS_ADD_CHAR_EVT = 9,
    ESP_GATTS_ADD_CHAR_DESCR_EVT = 10,
    ESP_GATTS_DELETE_EVT = 11,
    ESP_GATTS_START_EVT = 12,
    ESP_GATTS_STOP_EVT = 13,
    ESP_GATTS_CONNECT_EVT = 14,
    ESP_GATTS_DISCONNECT_EVT = 15,
    ESP_GATTS_OPEN_EVT = 16,
    ESP_GATTS_CANCEL_OPEN_EVT = 17,
    ESP_GATTS_CLOSE_EVT = 18,
    ESP_GATTS_LISTEN_EVT = 19,
    ESP_GATTS_CONGEST_EVT = 20,
} esp_gatts_cb_event_t;

typedef union {
    struct {
        esp_gatt_status_t status;
        uint16_t app_id;
    } reg;
    struct {
        uint16_t conn_id;
        uint32_t trans_id;
        uint8_t remote_bda[6];
        uint16_t handle;
        uint16_t offset;
        bool is_long;
        bool need_rsp;
    } read;
    struct {
        uint16_t conn_id;
        uint32_t trans_id;
        uint8_t remote_bda[6];
        uint16_t handle;
        uint16_t offset;
        bool need_rsp;
        bool is_prep;
        uint16_t len;
        uint8_t *value;
    } write;
    struct {
        esp_gatt_status_t status;
        uint16_t service_handle;
    } create;
    struct {
        esp_gatt_status_t status;
        uint16_t attr_handle;
        uint16_t service_handle;
        esp_bt_uuid_t char_uuid;
    } add_char;
    struct {
        esp_gatt_status_t status;
        uint16_t attr_handle;
        uint16_t service_handle;
        esp_bt_uuid_t descr_uuid;
    } add_char_descr;
    struct {
        esp_gatt_status_t status;
        uint16_t service_handle;
    } start;
    struct {
        esp_gatt_status_t status;
        uint16_t service_handle;
    } stop;
    struct {
        esp_gatt_status_t status;
        uint16_t service_handle;
    } del;
    struct {
        uint16_t conn_id;
        uint8_t link_role;
        uint8_t remote_bda[6];
    } connect;
    struct {
        uint16_t conn_id;
        uint8_t remote_bda[6];
        uint8_t reason;
    } disconnect;
    struct {
        esp_gatt_status_t status;
    } open;
    struct {
        esp_gatt_status_t status;
    } cancel_open;
    struct {
        esp_gatt_status_t status;
        uint16_t conn_id;
    } close;
} esp_ble_gatts_cb_param_t;

// From esp_gap_ble_api.h
typedef enum {
    ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT = 0,
    ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT,
    ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT,
    ESP_GAP_BLE_SCAN_RESULT_EVT,
    ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT,
    ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT,
    ESP_GAP_BLE_ADV_START_COMPLETE_EVT,
    ESP_GAP_BLE_SCAN_START_COMPLETE_EVT,
    ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT,
    ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT,
} esp_gap_ble_cb_event_t;

typedef uint8_t esp_bt_status_t;
#define ESP_BT_STATUS_SUCCESS 0

typedef struct {
    esp_bt_status_t status;
} esp_ble_gap_cb_param_adv_t;

typedef struct {
    esp_bt_status_t status;
    uint16_t min_int;
    uint16_t max_int;
    uint16_t conn_int;
    uint16_t latency;
    uint16_t timeout;
    uint8_t bda[6];
} esp_ble_gap_cb_param_conn_update_t;

typedef union {
    esp_ble_gap_cb_param_adv_t adv_start_cmpl;
    esp_ble_gap_cb_param_adv_t adv_stop_cmpl;
    esp_ble_gap_cb_param_adv_t adv_data_cmpl;
    esp_ble_gap_cb_param_adv_t scan_rsp_data_cmpl;
    esp_ble_gap_cb_param_conn_update_t update_conn_params;
} esp_ble_gap_cb_param_t;

typedef struct {
    bool set_scan_rsp;
    bool include_name;
    bool include_txpower;
    int min_interval;
    int max_interval;
    int appearance;
    uint16_t manufacturer_len;
    uint8_t *p_manufacturer_data;
    uint16_t service_data_len;
    uint8_t *p_service_data;
    uint16_t service_uuid_len;
    uint8_t *p_service_uuid;
    uint8_t flag;
} esp_ble_adv_data_t;

#define ESP_BLE_ADV_FLAG_GEN_DISC 0x02
#define ESP_BLE_ADV_FLAG_BREDR_NOT_SPT 0x04

typedef enum {
    ADV_TYPE_IND = 0,
} esp_ble_adv_type_t;

typedef enum {
    BLE_ADDR_TYPE_PUBLIC = 0,
} esp_ble_addr_type_t;

typedef enum {
    ADV_CHNL_ALL = 0x07,
} esp_ble_adv_channel_t;

typedef enum {
    ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY = 0,
} esp_ble_adv_filter_t;

typedef struct {
    uint16_t adv_int_min;
    uint16_t adv_int_max;
    esp_ble_adv_type_t adv_type;
    esp_ble_addr_type_t own_addr_type;
    uint8_t peer_addr[6];
    esp_ble_addr_type_t peer_addr_type;
    esp_ble_adv_channel_t channel_map;
    esp_ble_adv_filter_t adv_filter_policy;
} esp_ble_adv_params_t;

// Bluedroid status stub
typedef enum {
    ESP_BLUEDROID_STATUS_UNINITIALIZED = 0,
    ESP_BLUEDROID_STATUS_INITIALIZED,
    ESP_BLUEDROID_STATUS_ENABLED
} esp_bluedroid_status_t;

inline esp_bluedroid_status_t esp_bluedroid_get_status(void) {
    return ESP_BLUEDROID_STATUS_UNINITIALIZED;
}
