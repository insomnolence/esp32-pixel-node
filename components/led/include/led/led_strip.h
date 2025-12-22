#pragma once

#include "esp_err.h"
#include "driver/rmt_tx.h"
#include "led/led_strip_encoder.h"
#include "led/led_strip_interface.h"
#include <stdint.h>

// Include GPIO for indicator LED control during emergency shutdown
#if defined(CONFIG_BATTERY_MONITOR_ENABLED) && defined(CONFIG_INDICATOR_LED_ENABLED)
#include "driver/gpio.h"
#endif

// Forward declaration for battery monitor (optional dependency)
#ifdef CONFIG_BATTERY_MONITOR_ENABLED
class BatteryMonitor;
#endif

// ESP32 LED strip driver - replaces Arduino Adafruit_NeoPixel
class LEDStrip : public ILedStrip {
public:
    LEDStrip(uint16_t pixels, uint8_t pin, uint8_t type = 0);
    ~LEDStrip() override;

    // Core LED strip operations (compatible with Arduino Stripper interface)
    esp_err_t begin() override;
    esp_err_t show() override;
    void setPixelColor(uint16_t pixel, uint32_t color) override;
    void setPixelColor(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b) override;
    uint32_t getPixelColor(uint16_t pixel) const override;
    void setBrightness(uint8_t brightness) override;
    uint8_t getBrightness() const override;
    void clear() override;
    void clearAll(uint16_t totalPhysicalLEDs) override; // Clear entire physical strip
    uint16_t numPixels() const override;

    // Convenience methods from Arduino Stripper class
    void setAllColor(uint32_t color) override;
    void setAllFade(uint8_t fade_value);

    uint32_t getMillis() override;

    // Power limiting API
    void setMaxPowerMilliamps(uint32_t ma);
    uint32_t getMaxPowerMilliamps() const;
    void setSlewRateLimit(uint32_t ma_per_frame);
    uint32_t getSlewRateLimit() const;

#ifdef CONFIG_BATTERY_MONITOR_ENABLED
    // Voltage-aware power limiting
    void setBatteryMonitor(BatteryMonitor* monitor);
#endif

    // Static color utility functions (from Arduino Stripper)
    static uint32_t ColorFade(uint32_t color, uint8_t fade_value);
    static uint32_t ColorBlend(uint32_t color1, uint32_t color2, uint8_t blend_value);
    static uint32_t ColorRandom();
    static uint32_t ColorWheel(uint8_t wheel_pos);
    static uint32_t Color(uint8_t r, uint8_t g, uint8_t b);

private:
    rmt_channel_handle_t rmt_channel;
    rmt_encoder_handle_t led_encoder;
    rmt_transmit_config_t tx_config;

    const uint16_t pixel_count;    // Const - never changes after construction
    const uint8_t gpio_pin;        // Const - never changes after construction
    const uint8_t strip_type;      // Const - never changes after construction
    uint8_t brightness_level;
    uint32_t* pixel_buffer;
    uint8_t* grb_buffer;           // Pre-allocated GRB conversion buffer (pixel_count * 3 bytes)
    uint32_t max_power_ma;         // Power limit in milliamps (0 = disabled)
    uint32_t max_slew_ma;          // Max current increase per frame (slew rate limit)
    uint32_t previous_current_ma;  // Previous frame's total current (for slew limiting)

#ifdef CONFIG_BATTERY_MONITOR_ENABLED
    // Voltage-aware power limiting
    BatteryMonitor* battery_monitor;  // Optional battery monitor for voltage-based scaling
    uint16_t cached_voltage_mv;       // Cached battery voltage (updated every frame)
    uint32_t committed_ceiling_ma;    // Lowest ceiling committed to (ratchet-down, no flicker)
    uint32_t ceiling_reset_time_ms;   // Time when committed ceiling can reset
    bool emergency_shutdown;          // True when voltage critically low - LEDs off
    uint8_t indicator_blink_counter;  // Frame counter for indicator LED blinking
    static constexpr uint32_t CEILING_HOLD_TIME_MS = 10000; // Hold reduced ceiling for 10 seconds

    // Voltage thresholds (millivolts)
    static constexpr uint16_t VOLTAGE_EMERGENCY_ENTER = 3150;  // Enter emergency shutdown
    static constexpr uint16_t VOLTAGE_EMERGENCY_EXIT = 3300;   // Exit emergency (150mV hysteresis)
    static constexpr uint16_t VOLTAGE_VERY_LOW = 3200;         // 20% power
    static constexpr uint16_t VOLTAGE_LOW = 3300;              // 40% power
    static constexpr uint16_t VOLTAGE_MEDIUM_LOW = 3400;       // 60% power
    static constexpr uint16_t VOLTAGE_MEDIUM = 3500;           // 80% power
    // >= VOLTAGE_MEDIUM: 100% power

    // Indicator LED blinking: toggle every 7 frames (~100ms at 67fps = 5Hz blink)
    static constexpr uint8_t INDICATOR_BLINK_FRAMES = 7;
#endif

    // Internal helpers
    esp_err_t initRMT();
    uint32_t applyBrightness(uint32_t color) const;
    uint32_t calculateTotalCurrent() const;  // Calculate total current draw in mA
    uint8_t calculatePowerScale();           // Calculate scale factor with slew limiting
    uint32_t getEffectivePowerCeiling();     // Get voltage-adjusted power ceiling
};

// Color constants (from Arduino)
const uint32_t BLACK    = 0x000000;
const uint32_t RED      = 0xFF0000;
const uint32_t GREEN    = 0x00FF00;
const uint32_t BLUE     = 0x0000FF;
const uint32_t CYAN     = 0x00FFFF;
const uint32_t MAGENTA  = 0xFF00FF;
const uint32_t YELLOW   = 0xFFFF00;
const uint32_t WHITE    = 0xFFFFFF;

// Generic fade utility (from Arduino)
inline uint32_t fade(uint32_t low, uint32_t high, uint8_t value) {
    // Original: unsigned math causes underflow when high < low
    // return (high - low) * value / 255 + low;

    // Fixed: cast BEFORE subtraction to handle high < low correctly
    return ((int32_t)high - (int32_t)low) * value / 255 + low;
}