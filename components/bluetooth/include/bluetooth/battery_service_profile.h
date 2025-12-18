#pragma once

#include "bluetooth/gatt_profile.h"
#include "system/battery_monitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>

#ifdef CONFIG_BATTERY_MONITOR_ENABLED

class BatteryServiceProfile : public GattProfile {
public:
    BatteryServiceProfile(const std::string& service_uuid, const std::string& characteristic_uuid);
    ~BatteryServiceProfile();

    void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;

    // Battery monitor integration
    void setBatteryMonitor(BatteryMonitor* monitor);

    // Start/stop periodic updates
    void startPeriodicUpdates(uint32_t interval_ms = 10000);
    void stopPeriodicUpdates();

    // Manual battery update
    void sendBatteryUpdate();

private:
    void handleReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleCreateServiceEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleConnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleDisconnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleAddCharDescrEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;

    // Periodic update task
    static void batteryUpdateTask(void* pvParameters);
    void performBatteryUpdate();

    BatteryMonitor* battery_monitor;
    TaskHandle_t update_task_handle;
    uint32_t update_interval_ms;
    bool is_connected;
    uint16_t current_conn_id;
    esp_gatt_if_t current_gatts_if;

    static const char* TAG;
};

#endif // CONFIG_BATTERY_MONITOR_ENABLED
