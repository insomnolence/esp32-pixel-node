#include "gatt_profile.h"
#include "ble_gatt_server.h"
#include "ble_gap_handler.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "gap_gatt_data.h"
#include "packet.h"

#include <string.h> // For memset
#include <stdio.h>

const char* GattProfile::TAG = "GattProfile";

// Define the global instance of GattProfile
GattProfile gattProfile;

static uint8_t adv_service_uuid128[32] = {
    0x9e, 0xcc, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e,
    0x9e, 0xcc, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e,
};

GattProfile::GattProfile() : service_handle(0), a_property(0) {
    // Constructor
    adv_data = {
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
    scan_rsp_data = {
        .set_scan_rsp = true,
        .include_name = true,
        .include_txpower = true,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .service_uuid_len = sizeof(adv_service_uuid128),
        .p_service_uuid = adv_service_uuid128,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };
    const char *service_uuid_str = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
    my_service_uuid = convertStringToUuid(service_uuid_str);
    const char *characteristic_uuid_str = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
    my_characteristic_uuid = convertStringToUuid(characteristic_uuid_str);
}

GattProfile::~GattProfile() {
    // Destructor
}

void GattProfile::setServiceHandle(uint16_t service_handle) {
    this->service_handle = service_handle;
}

uint16_t GattProfile::getServiceHandle() {
    return service_handle;
}

esp_bt_uuid_t GattProfile::convertStringToUuid(const char *uuid_str) {
    esp_bt_uuid_t uuid;
    uuid.len = ESP_UUID_LEN_128;

    char uuid_str_no_hyphens[33];
    int j = 0;
    for (int i = 0; i < strlen(uuid_str); i++) {
        if (uuid_str[i] != '-') {
            uuid_str_no_hyphens[j++] = uuid_str[i];
        }
    }
    uuid_str_no_hyphens[j] = '\0';

    for (int i = 0; i < 32; i += 2) {
        sscanf(&uuid_str_no_hyphens[i], "%02hhx", &uuid.uuid.uuid128[15 - i / 2]);
    }
    return uuid;
}

void GattProfile::startAdvertising() {
    esp_err_t set_adv_data_ret = esp_ble_gap_config_adv_data(&adv_data);
    if (set_adv_data_ret) {
        ESP_LOGE(TAG, "config adv data failed, error code = %x", set_adv_data_ret);
    }
    adv_config_done |= ADV_CONFIG_FLAG;
    esp_err_t set_scan_rsp_data_ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (set_scan_rsp_data_ret) {
        ESP_LOGE(TAG, "config scan rsp data failed, error code = %x", set_scan_rsp_data_ret);
    }
    adv_config_done |= SCAN_RSP_CONFIG_FLAG;
}

void GattProfile::gattsProfileAEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (param == NULL) {
        return;
    }

    ESP_LOGI(TAG, "GATTS event %d", event);
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            profile_service_id.id.uuid = my_service_uuid;
            profile_service_id.id.inst_id = 0x00;
            profile_service_id.is_primary = true;

            esp_err_t create_service_ret = esp_ble_gatts_create_service(gatts_if, &profile_service_id, 4);
            if (create_service_ret) {
                ESP_LOGE(TAG, "create service failed, error code = %x", create_service_ret);
            }

            ESP_LOGI(TAG, "Advertising UUID:");
            ESP_LOG_BUFFER_HEX(TAG, adv_service_uuid128, 16);

            ESP_LOGI(TAG, "Service UUID (128-bit):");
            ESP_LOG_BUFFER_HEX(TAG, profile_service_id.id.uuid.uuid.uuid128, ESP_UUID_LEN_128);
            break;
        }
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
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
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
            if (!param->write.is_prep) {
                Packet received_pkt;
                memcpy(&received_pkt, param->write.value, sizeof(Packet));
                ESP_LOGI(TAG, "Received Packet: command=%u, brightness=%u, speed=%u, pattern=%u",
                         received_pkt.command, received_pkt.brightness, received_pkt.speed, received_pkt.pattern);
                ESP_LOGI(TAG, "Color: R=%" PRIu32 ", G=%" PRIu32 ", B=%" PRIu32, received_pkt.color[0], received_pkt.color[1], received_pkt.color[2]);
                ESP_LOGI(TAG, "Level: L1=%u, L2=%u, L3=%u", received_pkt.level[0], received_pkt.level[1], received_pkt.level[2]);
            }

            if (descr_handle == param->write.handle && param->write.len == 2) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
                        ESP_LOGI(TAG, "notify enable");
                    }
                } else if (descr_value == 0x0002) {
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE) {
                        ESP_LOGI(TAG, "indicate enable");
                    }
                } else if (descr_value == 0x0000) {
                    ESP_LOGI(TAG, "notify/indicate disable ");
                } else {
                    ESP_LOGE(TAG, "unknown descr value");
                }
            }
            break;
        }
        case ESP_GATTS_CREATE_EVT: {
            ESP_LOGI(TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
            setServiceHandle(param->create.service_handle);

            ESP_LOGI(TAG, "Advertising UUID:");
            ESP_LOG_BUFFER_HEX(TAG, adv_service_uuid128, 16);

            ESP_LOGI(TAG, "Service UUID (128-bit):");
            ESP_LOG_BUFFER_HEX(TAG, profile_service_id.id.uuid.uuid.uuid128, ESP_UUID_LEN_128);

            esp_err_t start_service_ret = esp_ble_gatts_start_service(getServiceHandle());
            if (start_service_ret) {
                ESP_LOGE(TAG, "start service failed, error code = %x", start_service_ret);
            } else {
                a_property = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;
                esp_err_t add_char_ret = esp_ble_gatts_add_char(getServiceHandle(), &my_characteristic_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                                a_property,
                                                                NULL, NULL);
                if (add_char_ret) {
                    ESP_LOGE(TAG, "add char failed, error code =%x", add_char_ret);
                }
            }
            break;
        }
        case ESP_GATTS_ADD_CHAR_EVT: {
            ESP_LOGI(TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
                     param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            char_handle = param->add_char.attr_handle;
            esp_bt_uuid_t char_client_config_uuid;
            char_client_config_uuid.len = ESP_UUID_LEN_16;
            char_client_config_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
            esp_err_t add_char_desc_ret = esp_ble_gatts_add_char_descr(getServiceHandle(), &char_client_config_uuid,
                                                                       ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                                       NULL, NULL);
            if (add_char_desc_ret) {
                ESP_LOGE(TAG, "add char descr failed, error code =%x", add_char_desc_ret);
            }
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
            ESP_LOGI(TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                     param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
            descr_handle = param->add_char_descr.attr_handle;
            break;
        }
        case ESP_GATTS_CONNECT_EVT: {
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            ESP_LOG_BUFFER_HEX(TAG, param->connect.remote_bda, 6);
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT: {
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, conn_id = %d", param->disconnect.conn_id);
            // Restart advertising after disconnection
            BLEGapHandler gapHandler;
            gapHandler.startAdvertising();
            break;
        }
        case ESP_GATTS_START_EVT: {
            ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                     param->start.status, param->start.service_handle);
            break;
        }
        case ESP_GATTS_STOP_EVT: {
            ESP_LOGI(TAG, "SERVICE_STOP_EVT, status %d, service_handle %d",
                     param->stop.status, param->stop.service_handle);
            break;
        }
        case ESP_GATTS_DELETE_EVT:
            ESP_LOGI(TAG, "SERVICE_DELETE_EVT, status %d, service_handle %d",
                     param->del.status, param->del.service_handle);
            break;
        case ESP_GATTS_OPEN_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_OPEN_EVT, status %d", param->open.status);
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CANCEL_OPEN_EVT, status %d", param->cancel_open.status);
            break;
        case ESP_GATTS_CLOSE_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CLOSE_EVT, conn_id %d", param->close.conn_id);
            break;
        case ESP_GATTS_LISTEN_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_LISTEN_EVT");
            break;
        case ESP_GATTS_CONGEST_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONGEST_EVT, conn_id %d, congest_status %d", param->congest.conn_id, param->congest.congested);
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        default:
            break;
    }
}
