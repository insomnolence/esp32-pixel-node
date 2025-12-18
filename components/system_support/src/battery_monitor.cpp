#include "system/battery_monitor.h"

#ifdef CONFIG_BATTERY_MONITOR_ENABLED

#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

const char* BatteryMonitor::TAG = "BatteryMonitor";

// Number of samples for oversampling (16x like HatTest)
#define ADC_OVERSAMPLE_COUNT 16

// ADC attenuation for full scale voltage range
#define ADC_ATTEN ADC_ATTEN_DB_12

BatteryMonitor::BatteryMonitor()
    : adc_handle(nullptr)
    , cali_handle(nullptr)
    , adc_channel(ADC_CHANNEL_0)
    , initialized(false)
    , cali_enabled(false)
    , r_high_kohm(CONFIG_BATTERY_DIVIDER_R_HIGH)
    , r_low_kohm(CONFIG_BATTERY_DIVIDER_R_LOW)
    , full_voltage_mv(CONFIG_BATTERY_FULL_MV)
    , empty_voltage_mv(CONFIG_BATTERY_EMPTY_MV)
{
}

BatteryMonitor::~BatteryMonitor() {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (cali_handle) {
        adc_cali_delete_scheme_curve_fitting(cali_handle);
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (cali_handle) {
        adc_cali_delete_scheme_line_fitting(cali_handle);
    }
#endif
    if (adc_handle) {
        adc_oneshot_del_unit(adc_handle);
    }
}

esp_err_t BatteryMonitor::init() {
    if (initialized) {
        return ESP_OK;
    }

    // Determine ADC unit and channel from GPIO
    int gpio_num = CONFIG_BATTERY_ADC_GPIO;
    adc_unit_t adc_unit;

#if CONFIG_IDF_TARGET_ESP32C3
    // ESP32-C3: ADC1 channels are GPIO0-4
    if (gpio_num <= 4) {
        adc_unit = ADC_UNIT_1;
        adc_channel = (adc_channel_t)gpio_num;
    } else {
        ESP_LOGE(TAG, "Invalid ADC GPIO %d for ESP32-C3", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
#elif CONFIG_IDF_TARGET_ESP32
    // ESP32: GPIO 35 is ADC1_CH7
    if (gpio_num == 35) {
        adc_unit = ADC_UNIT_1;
        adc_channel = ADC_CHANNEL_7;
    } else if (gpio_num >= 32 && gpio_num <= 39) {
        adc_unit = ADC_UNIT_1;
        adc_channel = (adc_channel_t)(gpio_num - 32);
    } else {
        ESP_LOGE(TAG, "Invalid ADC GPIO %d for ESP32", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
#else
    ESP_LOGE(TAG, "Unsupported target for battery monitoring");
    return ESP_ERR_NOT_SUPPORTED;
#endif

    // Initialize ADC unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = adc_unit,
#if CONFIG_IDF_TARGET_ESP32C3
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
#else
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
#endif
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure ADC channel
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ret = adc_oneshot_config_channel(adc_handle, adc_channel, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(adc_handle);
        adc_handle = nullptr;
        return ret;
    }

    // Initialize ADC calibration for accurate voltage readings
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = adc_unit,
        .chan = adc_channel,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
    if (ret == ESP_OK) {
        cali_enabled = true;
        ESP_LOGI(TAG, "ADC calibration enabled (curve fitting)");
    } else {
        ESP_LOGW(TAG, "ADC calibration failed, using uncalibrated readings");
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = adc_unit,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
        .default_vref = 1100,  // Default Vref in mV (used if eFuse not burned)
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
    if (ret == ESP_OK) {
        cali_enabled = true;
        ESP_LOGI(TAG, "ADC calibration enabled (line fitting)");
    } else {
        ESP_LOGW(TAG, "ADC calibration failed, using uncalibrated readings");
    }
#else
    ESP_LOGW(TAG, "No ADC calibration scheme supported, using uncalibrated readings");
#endif

    initialized = true;
    ESP_LOGI(TAG, "Battery monitor initialized on GPIO %d (ADC channel %d)", gpio_num, adc_channel);
    ESP_LOGI(TAG, "Voltage divider: %lu/%lu kOhm, range: %u-%u mV",
             r_high_kohm, r_low_kohm, empty_voltage_mv, full_voltage_mv);

    return ESP_OK;
}

uint32_t BatteryMonitor::readRawADC() {
    if (!initialized || !adc_handle) {
        return 0;
    }

    // Oversample for noise reduction
    uint32_t sum = 0;
    int raw_value;

    for (int i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
        if (adc_oneshot_read(adc_handle, adc_channel, &raw_value) == ESP_OK) {
            sum += raw_value;
        }
    }

    // Return average (right shift by 4 is equivalent to divide by 16)
    return sum >> 4;
}

uint16_t BatteryMonitor::rawToVoltage(uint32_t raw_value) {
    int adc_voltage_mv = 0;

    // Use calibration if available for accurate readings
    if (cali_enabled && cali_handle) {
        esp_err_t ret = adc_cali_raw_to_voltage(cali_handle, (int)raw_value, &adc_voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Calibration conversion failed, falling back to uncalibrated");
            adc_voltage_mv = 0;
        }
    }

    // Fallback to uncalibrated calculation if calibration not available or failed
    if (adc_voltage_mv == 0) {
        // Use conservative reference voltage estimate
        const uint32_t adc_ref_mv = 3100;
        const uint32_t adc_max = 4095;
        adc_voltage_mv = (raw_value * adc_ref_mv) / adc_max;
    }

    // Calculate actual battery voltage using voltage divider formula
    // Vbat = Vadc * (R_high + R_low) / R_low
    uint32_t battery_voltage_mv = (adc_voltage_mv * (r_high_kohm + r_low_kohm)) / r_low_kohm;

    return (uint16_t)battery_voltage_mv;
}

uint8_t BatteryMonitor::voltageToPercentage(uint16_t voltage_mv) {
    if (voltage_mv >= full_voltage_mv) {
        return 100;
    }
    if (voltage_mv <= empty_voltage_mv) {
        return 0;
    }

    // Linear interpolation between empty and full
    uint32_t range = full_voltage_mv - empty_voltage_mv;
    uint32_t offset = voltage_mv - empty_voltage_mv;

    return (uint8_t)((offset * 100) / range);
}

uint16_t BatteryMonitor::readVoltage() {
    uint32_t raw = readRawADC();
    return rawToVoltage(raw);
}

uint8_t BatteryMonitor::getPercentage() {
    uint16_t voltage = readVoltage();
    return voltageToPercentage(voltage);
}

BatteryStatus BatteryMonitor::getStatus() {
    BatteryStatus status;
    status.voltage_mv = readVoltage();
    status.percentage = voltageToPercentage(status.voltage_mv);
    status.is_charging = false; // TODO: Implement charging detection if hardware supports it
    return status;
}

#endif // CONFIG_BATTERY_MONITOR_ENABLED
