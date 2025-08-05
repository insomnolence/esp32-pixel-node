#ifndef BLE_GATT_SERVER_H_
#define BLE_GATT_SERVER_H_

#include "esp_err.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "gatt_profile.h" // Include for gatts_profile_inst_t
#include <functional> 
#include <vector>
#include <memory>

class BLEGattServer {
public:
    BLEGattServer();
    ~BLEGattServer();

    esp_err_t init();
    esp_err_t registerGattCallbacks();
    esp_err_t registerGattApp(uint16_t app_id);
    esp_err_t startAdvertising();
    esp_err_t setDeviceName(const char *device_name);
    void setProfileEventHandler(std::function<void(esp_gatts_cb_event_t, esp_gatt_if_t, esp_ble_gatts_cb_param_t*)> eventHandler, uint8_t profile_id); // Change here
    static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    void addProfile(std::shared_ptr<GattProfile> profile);
    void setAdvertisingCallback(std::function<void()> callback);

private:
    std::vector<std::shared_ptr<GattProfile>> profile_list;
    void handleGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    esp_err_t bleControllerInitAndEnable();
    esp_err_t bleBluedroidInitAndEnable();
    static const char* TAG;
    esp_ble_adv_params_t adv_params;
    static BLEGattServer* instance; // Declare the static instance variable
    std::function<void()> advertisingCallback;
};

#endif // BLE_GATT_SERVER_H_
