#include "gatt_profile.h"
#include "ble_gatt_server.h"
#include "ble_gap_handler.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "packet.h"

#include <string.h> // For memset
#include <stdio.h>

const char *GattProfile::TAG = "GattProfile";
uint16_t GattProfile::nextAppId = 0;

GattProfile::GattProfile(const std::string &service_uuid_str, const std::string &characteristic_uuid_str) : service_handle(0), a_property(0), app_id(nextAppId++)
{
    // Constructor
    my_service_uuid = convertStringToUuid(service_uuid_str.c_str());
    my_characteristic_uuid = convertStringToUuid(characteristic_uuid_str.c_str());

    // Initialize adv_service_uuid128 with the service UUID
    memcpy(adv_service_uuid128, my_service_uuid.uuid.uuid128, 16);

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
        .p_service_data = NULL,
        .service_uuid_len = sizeof(adv_service_uuid128),
        .p_service_uuid = adv_service_uuid128,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };
}

GattProfile::~GattProfile()
{
    // Destructor
}

void GattProfile::setServiceHandle(uint16_t service_handle)
{
    this->service_handle = service_handle;
}

uint16_t GattProfile::getServiceHandle()
{
    return service_handle;
}

esp_bt_uuid_t GattProfile::getServiceUuid()
{
    return my_service_uuid;
}

esp_bt_uuid_t GattProfile::getCharacteristicUuid()
{
    return my_characteristic_uuid;
}

uint16_t GattProfile::getCharHandle()
{
    return char_handle;
}

uint16_t GattProfile::getDescrHandle()
{
    return descr_handle;
}

uint16_t GattProfile::getAppId() const
{
    return app_id;
}

esp_gatt_if_t GattProfile::getGattIf()
{
    return gatt_interface;
}

void GattProfile::setGattIf(esp_gatt_if_t gatt_if)
{
    gatt_interface = gatt_if;
}

esp_bt_uuid_t GattProfile::convertStringToUuid(const char *uuid_str)
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

    for (int i = 0; i < 16; i++)
    {
        unsigned int byte;
        sscanf(&uuid_str_no_hyphens[i * 2], "%2x", &byte);
        uuid.uuid.uuid128[15 - i] = (uint8_t)byte;
    }
    return uuid;
}

void GattProfile::setAdvertisingCallback(std::function<void()> callback)
{
    advertisingCallback = callback;
}

void GattProfile::configureAdvertisingData()
{
    esp_err_t set_adv_data_ret = esp_ble_gap_config_adv_data(&adv_data);
    if (set_adv_data_ret)
    {
        ESP_LOGE(TAG, "config adv data failed, error code = %x", set_adv_data_ret);
    }
    esp_err_t set_scan_rsp_data_ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (set_scan_rsp_data_ret)
    {
        ESP_LOGE(TAG, "config scan rsp data failed, error code = %x", set_scan_rsp_data_ret);
    }
}

void GattProfile::handleCreateServiceEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    profile_service_id.id.uuid = my_service_uuid;
    profile_service_id.id.inst_id = 0x00;
    profile_service_id.is_primary = true;

    configureAdvertisingData();

    esp_err_t create_service_ret = esp_ble_gatts_create_service(gatts_if, &profile_service_id, 4);
    if (create_service_ret)
    {
        ESP_LOGE(TAG, "create service failed, error code = %x", create_service_ret);
    }

    ESP_LOGI(TAG, "Advertising UUID:");
    ESP_LOG_BUFFER_HEX(TAG, adv_service_uuid128, 16);

    ESP_LOGI(TAG, "Service UUID (128-bit):");
    ESP_LOG_BUFFER_HEX(TAG, profile_service_id.id.uuid.uuid.uuid128, ESP_UUID_LEN_128);
}

void GattProfile::handleReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
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
}

void GattProfile::handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
    if (!param->write.is_prep)
    {
        Packet received_pkt;
        memcpy(&received_pkt, param->write.value, sizeof(Packet));
        ESP_LOGI(TAG, "Received Packet: command=%u, brightness=%u, speed=%u, pattern=%u",
                 received_pkt.command, received_pkt.brightness, received_pkt.speed, received_pkt.pattern);
        ESP_LOGI(TAG, "Color: R=%" PRIu32 ", G=%" PRIu32 ", B=%" PRIu32, received_pkt.color[0], received_pkt.color[1], received_pkt.color[2]);
        ESP_LOGI(TAG, "Level: L1=%u, L2=%u, L3=%u", received_pkt.level[0], received_pkt.level[1], received_pkt.level[2]);
    }

    if (descr_handle == param->write.handle && param->write.len == 2)
    {
        uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
        if (descr_value == 0x0001)
        {
            if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
            {
                ESP_LOGI(TAG, "notify enable");
            }
        }
        else if (descr_value == 0x0002)
        {
            if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
            {
                ESP_LOGI(TAG, "indicate enable");
            }
        }
        else if (descr_value == 0x0000)
        {
            ESP_LOGI(TAG, "notify/indicate disable ");
        }
        else
        {
            ESP_LOGE(TAG, "unknown descr value");
        }
    }
}

void GattProfile::handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
             param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
    char_handle = param->add_char.attr_handle;
    esp_bt_uuid_t char_client_config_uuid;
    char_client_config_uuid.len = ESP_UUID_LEN_16;
    char_client_config_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
    esp_err_t add_char_desc_ret = esp_ble_gatts_add_char_descr(getServiceHandle(), &char_client_config_uuid,
                                                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                               NULL, NULL);
    if (add_char_desc_ret)
    {
        ESP_LOGE(TAG, "add char descr failed, error code =%x", add_char_desc_ret);
    }
}

void GattProfile::handleAddCharDescrEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
             param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
    descr_handle = param->add_char_descr.attr_handle;
}

void GattProfile::handleConnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
    ESP_LOG_BUFFER_HEX(TAG, param->connect.remote_bda, 6);
}

void GattProfile::handleDisconnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, conn_id = %d", param->disconnect.conn_id);
    // Check if callback is set, then use it to re-start the advertising.
    if (advertisingCallback)
    {
        advertisingCallback();
    }
}

void GattProfile::handleStartEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d",
             param->start.status, param->start.service_handle);
}

void GattProfile::handleStopEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "SERVICE_STOP_EVT, status %d, service_handle %d",
             param->stop.status, param->stop.service_handle);
}

void GattProfile::handleDeleteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "SERVICE_DELETE_EVT, status %d, service_handle %d",
             param->del.status, param->del.service_handle);
}

void GattProfile::handleOpenEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "ESP_GATTS_OPEN_EVT, status %d", param->open.status);
}

void GattProfile::handleCancelOpenEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "ESP_GATTS_CANCEL_OPEN_EVT, status %d", param->cancel_open.status);
}

void GattProfile::handleCloseEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "ESP_GATTS_CLOSE_EVT, conn_id %d", param->close.conn_id);
}

void GattProfile::handleListenEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "ESP_GATTS_LISTEN_EVT");
}

void GattProfile::handleCongestEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "ESP_GATTS_CONGEST_EVT, conn_id %d, congest_status %d", param->congest.conn_id, param->congest.congested);
}
