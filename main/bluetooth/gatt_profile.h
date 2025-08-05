#ifndef GATT_PROFILE_H_
#define GATT_PROFILE_H_

#include "esp_err.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include <string>
#include <functional>

class GattProfile {
public:
    GattProfile(const std::string& service_uuid_str, const std::string& characteristic_uuid_str);
    virtual ~GattProfile();

    // Pure virtual function to be implemented by subclasses
    virtual void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) = 0;

    void configureAdvertisingData();
    void setServiceHandle(uint16_t service_handle);
    uint16_t getServiceHandle();
    esp_bt_uuid_t getServiceUuid();
    esp_bt_uuid_t getCharacteristicUuid();
    uint16_t getCharHandle();
    uint16_t getDescrHandle();
    uint16_t getAppId() const;
    void setGattIf(esp_gatt_if_t gatts_if);
    esp_gatt_if_t getGattIf();
    void setAdvertisingCallback(std::function<void()> callback); 
    
protected:
    static const char* TAG;
    uint16_t service_handle;
    esp_gatt_srvc_id_t profile_service_id;
    esp_bt_uuid_t my_service_uuid;
    esp_bt_uuid_t my_characteristic_uuid;
    uint16_t char_handle;
    uint16_t descr_handle;
    esp_gatt_char_prop_t a_property;
    const uint16_t app_id;
    esp_ble_adv_data_t adv_data;
    esp_ble_adv_data_t scan_rsp_data;
    esp_gatt_if_t gatt_interface;
    std::function<void()> advertisingCallback;
    uint8_t adv_service_uuid128[16];

    static esp_bt_uuid_t convertStringToUuid(const char *uuid_str);
    virtual void handleCreateServiceEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleAddCharDescrEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleConnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleDisconnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleStartEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleStopEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleDeleteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleOpenEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleCancelOpenEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleCloseEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleListenEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    virtual void handleCongestEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

private:
    static uint16_t nextAppId;

};

#endif // GATT_PROFILE_H_
