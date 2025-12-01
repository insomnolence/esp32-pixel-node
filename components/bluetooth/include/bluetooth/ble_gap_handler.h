#pragma once

#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include "bluetooth/ble_hardware_interface.h"

class BLEGapHandler {
public:
    // Constructor with hardware interface for dependency injection (preferred for testability)
    explicit BLEGapHandler(BleHardwareInterface* hardware);
    ~BLEGapHandler();

    esp_err_t registerGapCallbacks();
    static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

private:
    BleHardwareInterface* hardware_; // Hardware abstraction layer
    static const char* TAG;
};
