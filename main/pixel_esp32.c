/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"

#define GATTS_TAG "GATTS_DEMO"

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define TEST_DEVICE_NAME "ESP_GATTS_DEMO"

static esp_gatt_char_prop_t a_property = 0;

static uint8_t adv_config_done = 0;
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)
/*
static uint8_t adv_service_uuid128[32] = {
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xEE,
    0x00,
    0x00,
    0x00,
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
};
*/
static uint8_t adv_service_uuid128[32] = {
    0x9e, 0xcc, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e,
    0x9e, 0xcc, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e,
//    0x6e, 0x40, 0x00, 0x01, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xcc, 0x9e,
//    0x6e, 0x40, 0x00, 0x02, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xcc, 0x9e,
}; 

esp_bt_uuid_t convert_string_to_uuid(const char *uuid_str)
{
    esp_bt_uuid_t uuid;
    uuid.len = ESP_UUID_LEN_128;

    char uuid_str_no_hyphens[33];
    int j = 0;
    for (int i = 0; i < strlen(uuid_str); i++)
    {
        if (uuid_str[i] != '-')
        {
            uuid_str_no_hyphens[j++] = uuid_str[i];
        }
    }
    uuid_str_no_hyphens[j] = '\0';

    for (int i = 0; i < 32; i += 2)
    {
        sscanf(&uuid_str_no_hyphens[i], "%02hhx", &uuid.uuid.uuid128[15 - i / 2]);
    }

    return uuid;
}

typedef struct
{
    uint8_t command;
    uint8_t brightness;
    uint8_t speed;
    uint8_t pattern;
    uint32_t color[3];
    uint8_t level[3];
} Packet;

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    uint16_t descr_handle;
};

static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising start failed");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed");
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully");
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

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (param == NULL)
    {
        return;
    }

    struct gatts_profile_inst *profile = &gl_profile_tab[PROFILE_A_APP_ID];

    switch (event)
    {
    case ESP_GATTS_REG_EVT:
    {
        gl_profile_tab[PROFILE_A_APP_ID].gatts_if = gatts_if; // Set gatts_if

        esp_gatt_srvc_id_t profile_service_id;
        const char *service_uuid_str = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
        esp_bt_uuid_t my_service_uuid = convert_string_to_uuid(service_uuid_str);
        profile_service_id.id.uuid = my_service_uuid;
        profile_service_id.id.inst_id = 0x00;
        profile_service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id = profile_service_id; 

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        if (set_dev_name_ret)
        {
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
        esp_err_t set_adv_data_ret = esp_ble_gap_config_adv_data(&adv_data);
        if (set_adv_data_ret)
        {
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", set_adv_data_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t set_scan_rsp_data_ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (set_scan_rsp_data_ret)
        {
            ESP_LOGE(GATTS_TAG, "config scan rsp data failed, error code = %x", set_scan_rsp_data_ret);
        }
        adv_config_done |= scan_rsp_config_flag;
        esp_err_t create_service_ret = esp_ble_gatts_create_service(gatts_if, &profile->service_id, 4);
        if (create_service_ret)
        {
            ESP_LOGE(GATTS_TAG, "create service failed, error code = %x", create_service_ret);
        }

        ESP_LOGI(GATTS_TAG, "Advertising UUID:");
        esp_log_buffer_hex(GATTS_TAG, adv_service_uuid128, 16);

        ESP_LOGI(GATTS_TAG, "Service UUID (128-bit):");
        esp_log_buffer_hex(GATTS_TAG, gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid128, ESP_UUID_LEN_128);
    }
    break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    case ESP_GATTS_WRITE_EVT:
    {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep)
        {
            Packet received_pkt;
            memcpy(&received_pkt, param->write.value, sizeof(Packet));
            ESP_LOGI(GATTS_TAG, "Received Packet: command=%u, brightness=%u, speed=%u, pattern=%u",
                     received_pkt.command, received_pkt.brightness, received_pkt.speed, received_pkt.pattern);
            ESP_LOGI(GATTS_TAG, "Color: R=%" PRIu32 ", G=%" PRIu32 ", B=%" PRIu32, received_pkt.color[0], received_pkt.color[1], received_pkt.color[2]);
            ESP_LOGI(GATTS_TAG, "Level: L1=%u, L2=%u, L3=%u", received_pkt.level[0], received_pkt.level[1], received_pkt.level[2]);
        }

        if (profile->descr_handle == param->write.handle && param->write.len == 2)
        {
            uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
            if (descr_value == 0x0001)
            {
                if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                {
                    ESP_LOGI(GATTS_TAG, "notify enable");
                }
            }
            else if (descr_value == 0x0002)
            {
                if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
                {
                    ESP_LOGI(GATTS_TAG, "indicate enable");
                }
            }
            else if (descr_value == 0x0000)
            {
                ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
            }
            else
            {
                ESP_LOGE(GATTS_TAG, "unknown descr value");
            }
        }
        break;
    }
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
        profile->service_handle = param->create.service_handle;

        const char *characteristic_uuid_str = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"; // characteristic UUID string
        esp_bt_uuid_t my_characteristic_uuid = convert_string_to_uuid(characteristic_uuid_str);
        profile->char_uuid = my_characteristic_uuid;

        ESP_LOGI(GATTS_TAG, "Advertising UUID:");
        esp_log_buffer_hex(GATTS_TAG, adv_service_uuid128, 16);

        ESP_LOGI(GATTS_TAG, "Service UUID (128-bit):");
        esp_log_buffer_hex(GATTS_TAG, gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid128, ESP_UUID_LEN_128);

        esp_err_t start_service_ret = esp_ble_gatts_start_service(profile->service_handle);
        if (start_service_ret)
        {
            ESP_LOGE(GATTS_TAG, "start service failed, error code = %x", start_service_ret);
        }
        else
        {
            a_property = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;
            esp_err_t add_char_ret = esp_ble_gatts_add_char(profile->service_handle, &profile->char_uuid,
                                                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                            a_property,
                                                            NULL, NULL);
            if (add_char_ret)
            {
                ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
            }
        }
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
    {
        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        profile->char_handle = param->add_char.attr_handle;
        profile->char_uuid.len = ESP_UUID_LEN_16;
        profile->char_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t add_char_desc_ret = esp_ble_gatts_add_char_descr(profile->service_handle, &profile->char_uuid,
                                                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                                   NULL, NULL);
        if (add_char_desc_ret)
        {
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_char_desc_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        profile->descr_handle = param->add_char_descr.attr_handle;
        break;
    case ESP_GATTS_CONNECT_EVT:
    {
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        esp_log_buffer_hex(GATTS_TAG, param->connect.remote_bda, 6);
        profile->conn_id = param->connect.conn_id;
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, conn_id = %d", param->disconnect.conn_id);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_STOP_EVT, status %d, service_handle %d",
                 param->stop.status, param->stop.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_DELETE_EVT, status %d, service_handle %d",
                 param->del.status, param->del.service_handle);
        break;
    case ESP_GATTS_OPEN_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_OPEN_EVT, conn_id %d", profile->conn_id);
        break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CANCEL_OPEN_EVT, conn_id %d", profile->conn_id);
        break;
    case ESP_GATTS_CLOSE_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CLOSE_EVT, conn_id %d", profile->conn_id);
        break;
    case ESP_GATTS_LISTEN_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_LISTEN_EVT");
        break;
    case ESP_GATTS_CONGEST_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONGEST_EVT, conn_id %d, congest_status %d", param->congest.conn_id, param->congest.congested);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[PROFILE_A_APP_ID].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGE(GATTS_TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == gl_profile_tab[idx].gatts_if)
            {
                if (gl_profile_tab[idx].gatts_cb)
                {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void app_main(void)
{
    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
    if (set_dev_name_ret)
    {
        ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        return;
    }
/*
    esp_gatt_srvc_id_t profile_service_id;

    const char *service_uuid_str = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"; // service UUID string
    esp_bt_uuid_t my_service_uuid = convert_string_to_uuid(service_uuid_str);

    profile_service_id.id.uuid = my_service_uuid;
    profile_service_id.id.inst_id = 0x00;
    profile_service_id.is_primary = true;

    gl_profile_tab[PROFILE_A_APP_ID].service_id = profile_service_id;
*/
    return;
}