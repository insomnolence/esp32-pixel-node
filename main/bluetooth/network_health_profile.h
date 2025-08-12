#ifndef NETWORK_HEALTH_PROFILE_H_
#define NETWORK_HEALTH_PROFILE_H_

#include "gatt_profile.h"
#include "../system/network_health.h"
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class NetworkHealthProfile : public GattProfile {
public:
    NetworkHealthProfile(const std::string& service_uuid_str, const std::string& characteristic_uuid_str);
    ~NetworkHealthProfile();

    void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    
    // Network health integration
    void setNetworkHealthMonitor(NetworkHealthMonitor* monitor);
    void startPeriodicUpdates(uint32_t interval_ms = 10000); // Default 10 seconds
    void stopPeriodicUpdates();
    
    // Manual health update
    void sendHealthUpdate();

private:
    void handleReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleCreateServiceEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleDisconnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleAddCharDescrEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleConnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    
    // Periodic update task
    static void healthUpdateTask(void* pvParameters);
    void performHealthUpdate();
    
    NetworkHealthMonitor* health_monitor;
    TaskHandle_t update_task_handle;
    uint32_t update_interval_ms;
    bool is_connected;
    uint16_t current_conn_id;
    esp_gatt_if_t current_gatts_if;
    
    static const char* TAG;
};

#endif // NETWORK_HEALTH_PROFILE_H_