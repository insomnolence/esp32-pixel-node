#include "network_health_profile.h"
#include "esp_log.h"
#include "esp_gatts_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

const char* NetworkHealthProfile::TAG = "NetworkHealthProfile";

NetworkHealthProfile::NetworkHealthProfile(const std::string& service_uuid_str, const std::string& characteristic_uuid_str)
    : GattProfile(service_uuid_str, characteristic_uuid_str)
    , health_monitor(nullptr)
    , update_task_handle(nullptr)
    , update_interval_ms(10000)
    , is_connected(false)
    , current_conn_id(0)
    , current_gatts_if(ESP_GATT_IF_NONE) {
}

NetworkHealthProfile::~NetworkHealthProfile() {
    stopPeriodicUpdates();
}

void NetworkHealthProfile::gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(TAG, "NetworkHealthProfile GATTS event %d", event);
    
    if (param == NULL) {
        return;
    }

    switch (event) {
        case ESP_GATTS_REG_EVT:
            // Create service using base class method (this triggers CREATE_EVT)
            ESP_LOGI(TAG, "📋 NetworkHealth service registration - creating service");
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
            ESP_LOGI(TAG, "🔗 Health analytics client connected!");
            handleConnectEvent(gatts_if, param);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "🔌 Health analytics client disconnected");
            handleDisconnectEvent(gatts_if, param);
            break;
        default:
            break;
    }
}

void NetworkHealthProfile::setNetworkHealthMonitor(NetworkHealthMonitor* monitor) {
    health_monitor = monitor;
    ESP_LOGI(TAG, "📊 Network health monitor attached to BLE profile");
}

void NetworkHealthProfile::startPeriodicUpdates(uint32_t interval_ms) {
    if (update_task_handle != nullptr) {
        ESP_LOGW(TAG, "⚠️ Periodic updates already running, stopping existing task");
        stopPeriodicUpdates();
    }
    
    update_interval_ms = interval_ms;
    
    xTaskCreate(
        healthUpdateTask,
        "health_updates",
        4096,  // Stack size
        this,  // Task parameter (this instance)
        5,     // Priority
        &update_task_handle
    );
    
    ESP_LOGI(TAG, "🔄 Started periodic health updates (every %lu ms)", interval_ms);
}

void NetworkHealthProfile::stopPeriodicUpdates() {
    if (update_task_handle != nullptr) {
        vTaskDelete(update_task_handle);
        update_task_handle = nullptr;
        ESP_LOGI(TAG, "🛑 Stopped periodic health updates");
    }
}

void NetworkHealthProfile::sendHealthUpdate() {
    if (!is_connected || health_monitor == nullptr) {
        ESP_LOGD(TAG, "⚠️ Cannot send health update: connected=%d, monitor=%p", is_connected, health_monitor);
        return;
    }
    
    NetworkHealth health = health_monitor->getCurrentHealth();
    
    // Send health data as notification
    esp_err_t ret = esp_ble_gatts_send_indicate(
        current_gatts_if,
        current_conn_id,
        char_handle,
        sizeof(NetworkHealth),
        (uint8_t*)&health,
        false  // false = notification, true = indication
    );
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "📡 Health update sent - Score: %d%%, Neighbors: %d, Success: %d%%", 
                 health.overall_score, health.active_neighbors, health.packet_success_rate);
    } else {
        ESP_LOGW(TAG, "❌ Failed to send health update: %s", esp_err_to_name(ret));
    }
}

void NetworkHealthProfile::handleReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(TAG, "📖 Health data read request");
    
    if (health_monitor == nullptr) {
        ESP_LOGW(TAG, "⚠️ No health monitor available for read");
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                   ESP_GATT_NOT_FOUND, nullptr);
        return;
    }
    
    NetworkHealth health = health_monitor->getCurrentHealth();
    
    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = param->read.handle;
    rsp.attr_value.len = sizeof(NetworkHealth);
    memcpy(rsp.attr_value.value, &health, sizeof(NetworkHealth));
    
    esp_err_t ret = esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                               ESP_GATT_OK, &rsp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to send health read response: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "✅ Health data read response sent - Score: %d%%", health.overall_score);
    }
}

void NetworkHealthProfile::handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    // Check if this is a write to the CCCD descriptor (for notification enable/disable)
    if (param->write.handle == descr_handle) {
        ESP_LOGI(TAG, "📝 CCCD descriptor write - enabling/disabling notifications");
        // Allow writes to CCCD descriptor for notification control
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                   ESP_GATT_OK, nullptr);
    } else {
        // Reject writes to the actual characteristic data (it's read-only)
        ESP_LOGW(TAG, "⚠️ Health characteristic data is read-only, ignoring write request");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                   ESP_GATT_WRITE_NOT_PERMIT, nullptr);
    }
}

void NetworkHealthProfile::handleConnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    is_connected = true;
    current_conn_id = param->connect.conn_id;
    current_gatts_if = gatts_if;
    
    // Send immediate health update upon connection
    if (health_monitor != nullptr) {
        sendHealthUpdate();
        ESP_LOGI(TAG, "📡 Sent immediate health update on BLE connect");
    }
    
    // Start periodic updates when client connects
    startPeriodicUpdates(update_interval_ms);
    
    ESP_LOGI(TAG, "🔄 Started health analytics streaming");
}

void NetworkHealthProfile::handleDisconnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    is_connected = false;
    current_conn_id = 0;
    current_gatts_if = ESP_GATT_IF_NONE;
    
    // Stop periodic updates when client disconnects
    stopPeriodicUpdates();
    
    ESP_LOGI(TAG, "🛑 Stopped health analytics streaming");
}

void NetworkHealthProfile::handleCreateServiceEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    // Prevent duplicate service creation - only handle if service handle not already set
    if (getServiceHandle() != 0) {
        ESP_LOGW(TAG, "⚠️ NetworkHealth service already created, skipping duplicate CREATE_EVT");
        return;
    }
    
    // Set the service handle from the CREATE event
    setServiceHandle(param->create.service_handle);
    
    // Start the NetworkHealth service (critical for service discovery)
    esp_err_t start_service_ret = esp_ble_gatts_start_service(getServiceHandle());
    ESP_LOGI(TAG, "🚀 NetworkHealth service start result: %d", start_service_ret);
    if (start_service_ret) {
        ESP_LOGE(TAG, "❌ NetworkHealth service start failed, error code = %x", start_service_ret);
        return; // Exit early on failure to prevent further issues
    } 
    
    ESP_LOGI(TAG, "✅ NetworkHealth service started successfully - now discoverable!");
    
    // Add health characteristic with Read + Notify properties and proper permissions
    a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
    esp_err_t add_char_ret = esp_ble_gatts_add_char(getServiceHandle(), &my_characteristic_uuid,
                                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                    a_property, NULL, NULL);
    if (add_char_ret) {
        ESP_LOGE(TAG, "❌ Add NetworkHealth characteristic failed, error code = %x", add_char_ret);
    } else {
        ESP_LOGI(TAG, "✅ NetworkHealth characteristic added successfully");
    }
}

void NetworkHealthProfile::handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    GattProfile::handleAddCharEvent(gatts_if, param);
    
    // Enable notifications for this characteristic
    if (param->add_char.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "🔔 Health characteristic created with notifications enabled");
    }
}

void NetworkHealthProfile::handleAddCharDescrEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    GattProfile::handleAddCharDescrEvent(gatts_if, param);
}

// Static task function
void NetworkHealthProfile::healthUpdateTask(void* pvParameters) {
    NetworkHealthProfile* profile = static_cast<NetworkHealthProfile*>(pvParameters);
    
    ESP_LOGI(TAG, "🔄 Health update task started");
    
    while (true) {
        profile->performHealthUpdate();
        vTaskDelay(pdMS_TO_TICKS(profile->update_interval_ms));
    }
}

void NetworkHealthProfile::performHealthUpdate() {
    if (is_connected && health_monitor != nullptr) {
        sendHealthUpdate();
    }
}