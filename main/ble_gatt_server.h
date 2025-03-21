#ifndef BLE_GATT_SERVER_H_
#define BLE_GATT_SERVER_H_

#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "gatt_profile.h" // Include for gatts_profile_inst_t
#include <functional> // Include for std::function

class GattsProfileInst {
public:
    GattsProfileInst();
    ~GattsProfileInst();

    void setGattCallback(std::function<void(esp_gatts_cb_event_t, esp_gatt_if_t, esp_ble_gatts_cb_param_t*)> callback); // Change here
    void setGattIf(esp_gatt_if_t gatts_if);
    std::function<void(esp_gatts_cb_event_t, esp_gatt_if_t, esp_ble_gatts_cb_param_t*)> getGattCallback(); // Change here
    esp_gatt_if_t getGattIf();
    
private:
    std::function<void(esp_gatts_cb_event_t, esp_gatt_if_t, esp_ble_gatts_cb_param_t*)> gatts_cb; // Change here
    esp_gatt_if_t gatts_if;
};

class BLEGattServer {
public:
    BLEGattServer();
    ~BLEGattServer();

    esp_err_t init();
    esp_err_t registerGattCallbacks();
    esp_err_t registerGattApp(uint16_t app_id);
    esp_err_t setDeviceName(const char *device_name);
    void setProfileEventHandler(std::function<void(esp_gatts_cb_event_t, esp_gatt_if_t, esp_ble_gatts_cb_param_t*)> eventHandler, uint8_t profile_id); // Change here
    static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

private:
    GattsProfileInst gl_profile_tab[PROFILE_NUM];
    void handleGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    esp_err_t bleControllerInitAndEnable();
    esp_err_t bleBluedroidInitAndEnable();
    static const char* TAG;
    static BLEGattServer* instance; // Declare the static instance variable
};

#endif // BLE_GATT_SERVER_H_
