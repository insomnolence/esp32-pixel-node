#include "bluetooth/battery_service_profile.h"

#ifdef CONFIG_BATTERY_MONITOR_ENABLED

#include "esp_log.h"
#include "esp_gatts_api.h"
#include <cstring>

const char* BatteryServiceProfile::TAG = "BatteryService";

BatteryServiceProfile::BatteryServiceProfile(const std::string& service_uuid, const std::string& characteristic_uuid)
    : GattProfile(service_uuid, characteristic_uuid)
    , battery_monitor(nullptr)
    , update_task_handle(nullptr)
    , update_interval_ms(CONFIG_BATTERY_UPDATE_INTERVAL_MS)
    , is_connected(false)
    , current_conn_id(0)
    , current_gatts_if(ESP_GATT_IF_NONE) {
}

BatteryServiceProfile::~BatteryServiceProfile() {
    stopPeriodicUpdates();
}

void BatteryServiceProfile::gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGD(TAG, "BatteryServiceProfile GATTS event %d", event);

    if (param == NULL) {
        return;
    }

    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "🔋 Battery service registration - creating service");
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
            ESP_LOGI(TAG, "🔗 Battery service client connected");
            handleConnectEvent(gatts_if, param);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "🔌 Battery service client disconnected");
            handleDisconnectEvent(gatts_if, param);
            break;
        default:
            break;
    }
}

void BatteryServiceProfile::setBatteryMonitor(BatteryMonitor* monitor) {
    battery_monitor = monitor;
    ESP_LOGI(TAG, "🔋 Battery monitor attached to BLE service");
}

void BatteryServiceProfile::startPeriodicUpdates(uint32_t interval_ms) {
    if (update_task_handle != nullptr) {
        ESP_LOGW(TAG, "⚠️ Periodic updates already running, stopping existing task");
        stopPeriodicUpdates();
    }

    update_interval_ms = interval_ms;

    xTaskCreate(
        batteryUpdateTask,
        "battery_updates",
        2048,  // Stack size (smaller than health updates)
        this,
        4,     // Priority
        &update_task_handle
    );

    ESP_LOGI(TAG, "🔄 Started periodic battery updates (every %lu ms)", (unsigned long)interval_ms);
}

void BatteryServiceProfile::stopPeriodicUpdates() {
    if (update_task_handle != nullptr) {
        vTaskDelete(update_task_handle);
        update_task_handle = nullptr;
        ESP_LOGI(TAG, "🛑 Stopped periodic battery updates");
    }
}

void BatteryServiceProfile::sendBatteryUpdate() {
    if (!is_connected || battery_monitor == nullptr || !battery_monitor->isAvailable()) {
        ESP_LOGD(TAG, "⚠️ Cannot send battery update: connected=%d, monitor=%p", is_connected, battery_monitor);
        return;
    }

    // Get battery status from monitor
    BatteryStatus status = battery_monitor->getStatus();

    // Pack into 4-byte struct: voltage_mv (2 bytes LE) + percentage (1 byte) + is_charging (1 byte)
    uint8_t battery_data[4];
    battery_data[0] = status.voltage_mv & 0xFF;         // voltage low byte
    battery_data[1] = (status.voltage_mv >> 8) & 0xFF;  // voltage high byte
    battery_data[2] = status.percentage;
    battery_data[3] = status.is_charging ? 1 : 0;

    // Send battery status as notification (4 bytes)
    esp_err_t ret = esp_ble_gatts_send_indicate(
        current_gatts_if,
        current_conn_id,
        char_handle,
        sizeof(battery_data),
        battery_data,
        false  // false = notification, true = indication
    );

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "🔋 Battery update sent - Voltage: %umV, Level: %d%%", status.voltage_mv, status.percentage);
    } else {
        ESP_LOGW(TAG, "❌ Failed to send battery update: %s", esp_err_to_name(ret));
    }
}

void BatteryServiceProfile::handleReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(TAG, "📖 Battery status read request");

    // Pack 4-byte struct: voltage_mv (2 bytes LE) + percentage (1 byte) + is_charging (1 byte)
    uint8_t battery_data[4] = {0, 0, 0, 0};

    if (battery_monitor != nullptr && battery_monitor->isAvailable()) {
        BatteryStatus status = battery_monitor->getStatus();
        battery_data[0] = status.voltage_mv & 0xFF;         // voltage low byte
        battery_data[1] = (status.voltage_mv >> 8) & 0xFF;  // voltage high byte
        battery_data[2] = status.percentage;
        battery_data[3] = status.is_charging ? 1 : 0;
        ESP_LOGI(TAG, "🔋 Battery: %umV (%d%%)", status.voltage_mv, status.percentage);
    } else {
        ESP_LOGW(TAG, "⚠️ Battery monitor not available, returning zeros");
    }

    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = param->read.handle;
    rsp.attr_value.len = 4;
    memcpy(rsp.attr_value.value, battery_data, 4);

    esp_err_t ret = esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                               ESP_GATT_OK, &rsp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to send battery read response: %s", esp_err_to_name(ret));
    } else {
        uint16_t voltage = battery_data[0] | (battery_data[1] << 8);
        ESP_LOGI(TAG, "✅ Battery read response sent: %umV", voltage);
    }
}

void BatteryServiceProfile::handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    // Check if this is a write to the CCCD descriptor (for notification enable/disable)
    if (param->write.handle == descr_handle) {
        ESP_LOGI(TAG, "📝 CCCD descriptor write - enabling/disabling notifications");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                   ESP_GATT_OK, nullptr);
    } else {
        // Battery level is read-only
        ESP_LOGW(TAG, "⚠️ Battery level is read-only, ignoring write request");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                   ESP_GATT_WRITE_NOT_PERMIT, nullptr);
    }
}

void BatteryServiceProfile::handleConnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    is_connected = true;
    current_conn_id = param->connect.conn_id;
    current_gatts_if = gatts_if;

    // Send immediate battery update upon connection
    if (battery_monitor != nullptr && battery_monitor->isAvailable()) {
        sendBatteryUpdate();
        ESP_LOGI(TAG, "📡 Sent immediate battery update on BLE connect");
    }

    // Start periodic updates when client connects
    startPeriodicUpdates(update_interval_ms);
}

void BatteryServiceProfile::handleDisconnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    is_connected = false;
    current_conn_id = 0;
    current_gatts_if = ESP_GATT_IF_NONE;

    // Stop periodic updates when client disconnects
    stopPeriodicUpdates();

    ESP_LOGI(TAG, "🛑 Stopped battery level streaming");
}

void BatteryServiceProfile::handleCreateServiceEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (getServiceHandle() != 0) {
        ESP_LOGW(TAG, "⚠️ Battery service already created, skipping duplicate CREATE_EVT");
        return;
    }

    setServiceHandle(param->create.service_handle);

    esp_err_t start_service_ret = esp_ble_gatts_start_service(getServiceHandle());
    ESP_LOGI(TAG, "🚀 Battery service start result: %d", start_service_ret);
    if (start_service_ret) {
        ESP_LOGE(TAG, "❌ Battery service start failed, error code = %x", start_service_ret);
        return;
    }

    ESP_LOGI(TAG, "✅ Battery service started successfully (UUID: 0x180F)");

    // Add battery level characteristic with Read + Notify properties
    a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
    esp_err_t add_char_ret = esp_ble_gatts_add_char(getServiceHandle(), &my_characteristic_uuid,
                                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                    a_property, NULL, NULL);
    if (add_char_ret) {
        ESP_LOGE(TAG, "❌ Add battery level characteristic failed, error code = %x", add_char_ret);
    } else {
        ESP_LOGI(TAG, "✅ Battery level characteristic added (UUID: 0x2A19)");
    }
}

void BatteryServiceProfile::handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    GattProfile::handleAddCharEvent(gatts_if, param);

    if (param->add_char.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "🔔 Battery level characteristic created with notifications enabled");
    }
}

void BatteryServiceProfile::handleAddCharDescrEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    GattProfile::handleAddCharDescrEvent(gatts_if, param);
}

// Static task function
void BatteryServiceProfile::batteryUpdateTask(void* pvParameters) {
    BatteryServiceProfile* profile = static_cast<BatteryServiceProfile*>(pvParameters);

    ESP_LOGI(TAG, "🔄 Battery update task started");

    while (true) {
        profile->performBatteryUpdate();
        vTaskDelay(pdMS_TO_TICKS(profile->update_interval_ms));
    }
}

void BatteryServiceProfile::performBatteryUpdate() {
    if (is_connected && battery_monitor != nullptr && battery_monitor->isAvailable()) {
        sendBatteryUpdate();
    }
}

#endif // CONFIG_BATTERY_MONITOR_ENABLED
