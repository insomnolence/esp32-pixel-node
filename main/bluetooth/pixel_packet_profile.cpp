#include "pixel_packet_profile.h"
#include "esp_log.h"
#include "packet.h"
#include "ble_gap_handler.h"
#include "string.h"
#include "esp_gatts_api.h" // For esp_ble_gatts_close
#include "esp_gap_ble_api.h" // For esp_ble_gap_disconnect

PixelPacketProfile::PixelPacketProfile(const std::string& service_uuid_str, const std::string& characteristic_uuid_str)
    : GattProfile(service_uuid_str, characteristic_uuid_str)
    , current_conn_id(0)
    , current_gatts_if(ESP_GATT_IF_NONE)
    , is_connected(false) {
}

PixelPacketProfile::~PixelPacketProfile() {
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
            ESP_LOGI(TAG, "🔥 BLE CLIENT CONNECTED!");
            handleConnectEvent(gatts_if, param);
            
            // Track connection state for potential forced disconnection
            current_conn_id = param->connect.conn_id;
            current_gatts_if = gatts_if;
            is_connected = true;
            
            if (ble_connection_callback) {
                ESP_LOGI(TAG, "🔥 Calling BLE connection callback (connected=true)");
                ble_connection_callback(true);
                ESP_LOGI(TAG, "🔥 BLE connection callback completed successfully");
            } else {
                ESP_LOGW(TAG, "🔥 No BLE connection callback set!");
            }
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "🔥 BLE CLIENT DISCONNECTED!");
            handleDisconnectEvent(gatts_if, param);
            
            // Clear connection state
            is_connected = false;
            current_conn_id = 0;
            current_gatts_if = ESP_GATT_IF_NONE;
            
            if (ble_connection_callback) {
                ESP_LOGI(TAG, "🔥 Calling BLE connection callback (connected=false)");
                ble_connection_callback(false);
                ESP_LOGI(TAG, "🔥 BLE disconnection callback completed successfully");
            } else {
                ESP_LOGW(TAG, "🔥 No BLE connection callback set!");
            }
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
    ESP_LOGI(TAG, "🔥 PixelPacketProfile GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
    ESP_LOGI(TAG, "🔥 Write details: is_prep=%d, len=%d, need_rsp=%d", param->write.is_prep, param->write.len, param->write.need_rsp);
    
    if (!param->write.is_prep && param->write.len > 0) {
        ESP_LOGI(TAG, "🔥 Processing write data...");
        
        // Create generic packet from BLE data (any format)
        GenericPacket received_packet(param->write.value, param->write.len);
        
        ESP_LOGI(TAG, "🔥 Received BLE packet: %zu bytes, valid=%d", received_packet.getLength(), received_packet.isValid());
        
        // Log packet contents in hex for debugging
        const uint8_t* data = received_packet.getData();
        ESP_LOG_BUFFER_HEX(TAG, data, received_packet.getLength());
        
        // Forward generic packet to mesh network if callback is set
        if (packet_forward_callback) {
            ESP_LOGI(TAG, "🔥 Forwarding packet to mesh network callback");
            if (received_packet.isValid()) {
                packet_forward_callback(received_packet);
                ESP_LOGI(TAG, "🔥 Packet forwarded successfully");
            } else {
                ESP_LOGW(TAG, "🔥 Packet not valid, not forwarding");
            }
        } else {
            ESP_LOGW(TAG, "🔥 No packet forward callback set!");
        }
    } else {
        ESP_LOGI(TAG, "🔥 Skipping write: is_prep=%d, len=%d", param->write.is_prep, param->write.len);
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

void PixelPacketProfile::setBleConnectionCallback(std::function<void(bool)> callback) {
    ble_connection_callback = callback;
}

void PixelPacketProfile::setPacketForwardCallback(std::function<void(const GenericPacket&)> callback) {
    packet_forward_callback = callback;
}

void PixelPacketProfile::forceDisconnect() {
    if (!is_connected) {
        ESP_LOGW(TAG, "🔒 Force disconnect called but no active connection");
        return;
    }
    
    ESP_LOGI(TAG, "🔒 Force disconnecting BLE connection (conn_id=%d)", current_conn_id);
    
    // Use esp_ble_gatts_close to forcefully close the GATT connection
    esp_err_t ret = esp_ble_gatts_close(current_gatts_if, current_conn_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "🔒 Failed to force disconnect: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "🔒 Force disconnect initiated successfully");
    }
}
