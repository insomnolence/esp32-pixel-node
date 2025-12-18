#pragma once

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include <stdint.h>

#ifdef CONFIG_BATTERY_MONITOR_ENABLED

// Battery status structure for BLE reporting
struct BatteryStatus {
    uint16_t voltage_mv;    // Battery voltage in millivolts
    uint8_t percentage;     // Battery percentage (0-100)
    bool is_charging;       // True if charging detected (future use)
} __attribute__((packed));

class BatteryMonitor {
public:
    BatteryMonitor();
    ~BatteryMonitor();

    // Initialize ADC for battery monitoring
    esp_err_t init();

    // Read current battery voltage (in millivolts)
    uint16_t readVoltage();

    // Get battery percentage (0-100)
    uint8_t getPercentage();

    // Get full battery status
    BatteryStatus getStatus();

    // Check if battery monitoring is available
    bool isAvailable() const { return initialized; }

private:
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    adc_channel_t adc_channel;
    bool initialized;
    bool cali_enabled;

    // Configuration from Kconfig
    uint32_t r_high_kohm;
    uint32_t r_low_kohm;
    uint16_t full_voltage_mv;
    uint16_t empty_voltage_mv;

    // Read raw ADC with oversampling
    uint32_t readRawADC();

    // Convert raw ADC to voltage considering divider
    uint16_t rawToVoltage(uint32_t raw_value);

    // Convert voltage to percentage
    uint8_t voltageToPercentage(uint16_t voltage_mv);

    static const char* TAG;
};

#endif // CONFIG_BATTERY_MONITOR_ENABLED
