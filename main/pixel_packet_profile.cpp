#include "pixel_packet_profile.h"
#include "esp_log.h"
#include "packet.h"
#include "ble_gap_handler.h"
#include "string.h"

PixelPacketProfile::PixelPacketProfile(const std::string& service_uuid_str, const std::string& characteristic_uuid_str)
    : GattProfile(service_uuid_str, characteristic_uuid_str) {
    // PixelPacketProfile specific constructor actions
}

PixelPacketProfile::~PixelPacketProfile() {
    // PixelPacketProfile specific destructor actions
}

void PixelPacketProfile::gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(TAG, "PixelPacketProfile GATTS event %d my_gatts_if: %d Passed gatts_if: %d", event, gatt_interface, gatts_if);
    
    if (param == NULL) {
        return;
    }

    ESP_LOGI(TAG, "PixelPacketProfile GATTS event %d", event);
    switch (event) {
        case ESP_GATTS_REG_EVT:
            GattProfile::handleCreateServiceEvent(gatts_if, param);
            break;
        case ESP_GATTS_READ_EVT:
            handleReadEvent(gatts_if, param);
            break;
        case ESP_GATTS_WRITE_EVT:
            handleWriteEvent(gatts_if, param);
            break;
        case ESP_GATTS_CREATE_EVT:
            handleCreateServiceEvent(gatts_if, param);
            break;
        case ESP_GATTS_ADD_CHAR_EVT:
            handleAddCharEvent(gatts_if, param);
            break;
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            handleAddCharDescrEvent(gatts_if, param);
            break;
        case ESP_GATTS_CONNECT_EVT:
            handleConnectEvent(gatts_if, param);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            handleDisconnectEvent(gatts_if, param);
            break;
        case ESP_GATTS_START_EVT:
            handleStartEvent(gatts_if, param);
            break;
        case ESP_GATTS_STOP_EVT:
            handleStopEvent(gatts_if, param);
            break;
        case ESP_GATTS_DELETE_EVT:
            handleDeleteEvent(gatts_if, param);
            break;
        case ESP_GATTS_OPEN_EVT:
            handleOpenEvent(gatts_if, param);
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            handleCancelOpenEvent(gatts_if, param);
            break;
        case ESP_GATTS_CLOSE_EVT:
            handleCloseEvent(gatts_if, param);
            break;
        case ESP_GATTS_LISTEN_EVT:
            handleListenEvent(gatts_if, param);
            break;
        case ESP_GATTS_CONGEST_EVT:
            handleCongestEvent(gatts_if, param);
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        default:
            break;
    }
}

void PixelPacketProfile::handleReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(TAG, "PixelPacketProfile GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
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

void PixelPacketProfile::handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(TAG, "PixelPacketProfile GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
    if (!param->write.is_prep) {
        Packet received_pkt;
        memcpy(&received_pkt, param->write.value, sizeof(Packet));
        ESP_LOGI(TAG, "Received Packet: command=%u, brightness=%u, speed=%u, pattern=%u",
                 received_pkt.command, received_pkt.brightness, received_pkt.speed, received_pkt.pattern);
        ESP_LOGI(TAG, "Color: R=%" PRIu32 ", G=%" PRIu32 ", B=%" PRIu32, received_pkt.color[0], received_pkt.color[1], received_pkt.color[2]);
        ESP_LOGI(TAG, "Level: L1=%u, L2=%u, L3=%u", received_pkt.level[0], received_pkt.level[1], received_pkt.level[2]);
    }

    if (getDescrHandle() == param->write.handle && param->write.len == 2) {
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
}

void PixelPacketProfile::handleCreateServiceEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param){
    // TBD: Might call this after this after the rest of this code is set up. 
    // Reconsider using indicators that the bluetooth service has "started advertising" so we don't 
    // send multiple messages.
    //GattProfile::handleCreateServiceEvent(gatts_if, param);
    setServiceHandle(param->create.service_handle);
    
    esp_err_t start_service_ret = esp_ble_gatts_start_service(getServiceHandle());
    ESP_LOGI(TAG, "handleCreateServiceEvent start service ret: %d", start_service_ret);
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
}

void PixelPacketProfile::handleDisconnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    GattProfile::handleDisconnectEvent(gatts_if, param);
}

void PixelPacketProfile::handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param){
    GattProfile::handleAddCharEvent(gatts_if, param);
}

void PixelPacketProfile::handleAddCharDescrEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param){
    GattProfile::handleAddCharDescrEvent(gatts_if, param);
}
