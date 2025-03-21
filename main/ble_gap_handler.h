#ifndef BLE_GAP_HANDLER_H_
#define BLE_GAP_HANDLER_H_

#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include "gap_gatt_data.h"

class BLEGapHandler {
public:
    BLEGapHandler();
    ~BLEGapHandler();

    esp_err_t registerGapCallbacks();
    void startAdvertising();
    static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

private:
    static const char* TAG;
};

#endif // BLE_GAP_HANDLER_H_
