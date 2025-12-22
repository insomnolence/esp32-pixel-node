#include "led/led_strip.h"
#include "led/led_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#ifdef CONFIG_BATTERY_MONITOR_ENABLED
#include "system/battery_monitor.h"
#endif

#include "esp_random.h"
#include "esp_check.h"
#include "driver/rmt_tx.h"
#include <cstring>
#include <cstdlib>
#include <new>

static const char* TAG = "LEDStrip";

// WS2812 timing constants (in nanoseconds)
#define WS2812_T0H_NS    350    // 0 bit high time
#define WS2812_T0L_NS    900    // 0 bit low time  
#define WS2812_T1H_NS    900    // 1 bit high time
#define WS2812_T1L_NS    350    // 1 bit low time
#define WS2812_RESET_US  50     // Reset time in microseconds

// RMT resolution (10MHz = 100ns per tick)
#define RMT_RESOLUTION_HZ 10000000

LEDStrip::LEDStrip(uint16_t pixels, uint8_t pin, uint8_t type)
    : rmt_channel(nullptr)
    , led_encoder(nullptr)
    , pixel_count(pixels)
    , gpio_pin(pin)
    , strip_type(type)
    , brightness_level(255)
    , pixel_buffer(nullptr)
    , grb_buffer(nullptr)
    , max_power_ma(LED_DEFAULT_MAX_POWER_MA)
    , max_slew_ma(LED_MAX_SLEW_MA_PER_FRAME)
    , previous_current_ma(0)
#ifdef CONFIG_BATTERY_MONITOR_ENABLED
    , battery_monitor(nullptr)
    , cached_voltage_mv(4200)  // Assume full battery initially
    , committed_ceiling_ma(0)  // 0 = not committed yet, will use max_power_ma
    , ceiling_reset_time_ms(0)
    , emergency_shutdown(false)
    , indicator_blink_counter(0)
#endif
{
    // Validate parameters
    if (pixels == 0) {
        ESP_LOGE(TAG, "Invalid pixel count: %d", pixels);
        return;
    }
    
    // Allocate pixel buffer with bounds checking
    pixel_buffer = new(std::nothrow) uint32_t[pixel_count];
    if (!pixel_buffer) {
        ESP_LOGE(TAG, "Failed to allocate pixel buffer for %d pixels", pixel_count);
        return;
    }
    memset(pixel_buffer, 0, pixel_count * sizeof(uint32_t));
    
    // Allocate GRB conversion buffer (persistent for lifetime)
    grb_buffer = new(std::nothrow) uint8_t[pixel_count * 3];
    if (!grb_buffer) {
        ESP_LOGE(TAG, "Failed to allocate GRB buffer for %d pixels", pixel_count);
        delete[] pixel_buffer;
        pixel_buffer = nullptr;
        return;
    }
    
    // Initialize transmit config
    tx_config = {
        .loop_count = 0, // no loop
        .flags = {},
    };
}

LEDStrip::~LEDStrip() {
    if (rmt_channel) {
        rmt_disable(rmt_channel);
        rmt_del_channel(rmt_channel);
    }
    if (led_encoder) {
        rmt_del_encoder(led_encoder);
    }
    delete[] pixel_buffer;
    delete[] grb_buffer;
}

esp_err_t LEDStrip::begin() {
    // Check if buffers were allocated successfully
    if (!pixel_buffer) {
        ESP_LOGE(TAG, "Cannot initialize LED strip: pixel buffer allocation failed");
        return ESP_ERR_NO_MEM;
    }
    if (!grb_buffer) {
        ESP_LOGE(TAG, "Cannot initialize LED strip: GRB buffer allocation failed");
        return ESP_ERR_NO_MEM;
    }
    
    esp_err_t ret = initRMT();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RMT: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "LED strip initialized: %d pixels on GPIO %d", pixel_count, gpio_pin);
    if (max_power_ma > 0) {
        ESP_LOGI(TAG, "Power limiting enabled: %lumA ceiling, %lumA/frame slew limit",
                 max_power_ma, max_slew_ma);
    } else {
        ESP_LOGI(TAG, "Power limiting disabled");
    }
    return ESP_OK;
}

esp_err_t LEDStrip::initRMT() {
    esp_err_t ret;
    
    // Create RMT TX channel
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = static_cast<gpio_num_t>(gpio_pin),
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64, // Increase if experiencing issues
        .trans_queue_depth = 4,
        .intr_priority = 0,
        .flags = {},
    };
    
    ret = rmt_new_tx_channel(&tx_chan_config, &rmt_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "create RMT TX channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create LED strip encoder
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_RESOLUTION_HZ,
    };
    ret = rmt_new_led_strip_encoder(&encoder_config, &led_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "create LED encoder failed: %s", esp_err_to_name(ret));
        // Clean up channel on encoder failure
        rmt_del_channel(rmt_channel);
        rmt_channel = nullptr;
        return ret;
    }

    // Enable RMT channel
    ret = rmt_enable(rmt_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "enable RMT channel failed: %s", esp_err_to_name(ret));
        // Clean up on enable failure
        rmt_del_encoder(led_encoder);
        led_encoder = nullptr;
        rmt_del_channel(rmt_channel);
        rmt_channel = nullptr;
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t LEDStrip::show() {
    if (!rmt_channel || !led_encoder || !pixel_buffer) {
        return ESP_ERR_INVALID_STATE;
    }

    // Convert RGB to GRB format for WS2812 (using pre-allocated buffer)
    if (!grb_buffer) {
        ESP_LOGE(TAG, "GRB buffer not allocated");
        return ESP_ERR_INVALID_STATE;
    }

    // Calculate power scale factor with slew rate limiting
    // (255 = no scaling, <255 = reduce brightness to stay within power/slew limits)
    // This also updates emergency_shutdown state based on voltage
    uint8_t power_scale = calculatePowerScale();

#ifdef CONFIG_BATTERY_MONITOR_ENABLED
    // Emergency shutdown: output all black and blink indicator LED
    if (emergency_shutdown) {
        // All LEDs off
        memset(grb_buffer, 0, pixel_count * 3);

#ifdef CONFIG_INDICATOR_LED_ENABLED
        // Blink indicator LED at 5Hz (toggle every INDICATOR_BLINK_FRAMES)
        if (++indicator_blink_counter >= INDICATOR_BLINK_FRAMES) {
            indicator_blink_counter = 0;
            static bool indicator_state = false;
            indicator_state = !indicator_state;
            gpio_set_level((gpio_num_t)CONFIG_INDICATOR_LED_GPIO, indicator_state ? 1 : 0);
        }
#endif

        // Transmit all-black data
        esp_err_t ret = rmt_transmit(rmt_channel, led_encoder, grb_buffer, pixel_count * 3, &tx_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "RMT transmit failed: %s", esp_err_to_name(ret));
        }
        return ret;
    } else {
#ifdef CONFIG_INDICATOR_LED_ENABLED
        // Normal operation: indicator LED off
        gpio_set_level((gpio_num_t)CONFIG_INDICATOR_LED_GPIO, 0);
        indicator_blink_counter = 0;
#endif
    }
#endif  // CONFIG_BATTERY_MONITOR_ENABLED

    for (uint16_t i = 0; i < pixel_count; i++) {
        uint32_t color = applyBrightness(pixel_buffer[i]);

        // Apply power scaling if needed
        if (power_scale < 255) {
            uint8_t r = (color >> 16) & 0xFF;
            uint8_t g = (color >> 8) & 0xFF;
            uint8_t b = color & 0xFF;

            uint16_t scale = power_scale + 1;
            r = (r * scale) >> 8;
            g = (g * scale) >> 8;
            b = (b * scale) >> 8;

            color = Color(r, g, b);
        }

        grb_buffer[i * 3 + 0] = (color >> 8) & 0xFF;  // Green
        grb_buffer[i * 3 + 1] = (color >> 16) & 0xFF; // Red
        grb_buffer[i * 3 + 2] = color & 0xFF;         // Blue
    }

    // Transmit data
    esp_err_t ret = rmt_transmit(rmt_channel, led_encoder, grb_buffer, pixel_count * 3, &tx_config);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMT transmit failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

uint32_t LEDStrip::calculateTotalCurrent() const {
    // Calculate total current draw based on pixel colors and brightness
    // Each color channel draws up to 20mA at full brightness (255)
    // Formula: channel_current = (channel_value / 255) * 20mA * (brightness / 255)
    //
    // To avoid floating point, we calculate in units of 0.1mA:
    // channel_current_01ma = channel_value * brightness * 200 / (255 * 255)

    uint32_t total_current_01ma = 0;
    uint16_t brightness_scale = brightness_level + 1;

    for (uint16_t i = 0; i < pixel_count; i++) {
        uint32_t color = pixel_buffer[i];
        uint8_t r = (color >> 16) & 0xFF;
        uint8_t g = (color >> 8) & 0xFF;
        uint8_t b = color & 0xFF;

        // Apply brightness scaling to get actual output values
        uint8_t r_out = (r * brightness_scale) >> 8;
        uint8_t g_out = (g * brightness_scale) >> 8;
        uint8_t b_out = (b * brightness_scale) >> 8;

        // Current in 0.1mA units: (value / 255) * 20mA * 10 = value * 200 / 255
        total_current_01ma += (r_out + g_out + b_out) * 200 / 255;
    }

    // Convert to milliamps
    return total_current_01ma / 10;
}

uint8_t LEDStrip::calculatePowerScale() {
    // If power limiting is disabled, return full scale
    if (max_power_ma == 0) {
        return 255;
    }

    // Get voltage-adjusted power ceiling (may be lower than max_power_ma if battery is low)
    uint32_t effective_ceiling = getEffectivePowerCeiling();

    // Calculate what the pattern wants
    uint32_t target_current_ma = calculateTotalCurrent();

    // Apply slew rate limiting: limit how fast current can INCREASE
    // (We don't limit decreases - those are safe and should be instant)
    uint32_t allowed_current_ma = target_current_ma;

    if (target_current_ma > previous_current_ma + max_slew_ma) {
        // Current increase exceeds slew limit - cap it
        allowed_current_ma = previous_current_ma + max_slew_ma;

        // Log when slew limiting is active (throttled)
        static uint32_t last_slew_log_time = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - last_slew_log_time > 1000) {  // Log at most every second
            ESP_LOGD(TAG, "Slew limit: %lumA target, %lumA allowed (prev=%lumA, max_slew=%lumA)",
                     target_current_ma, allowed_current_ma, previous_current_ma, max_slew_ma);
            last_slew_log_time = now;
        }
    }

    // Apply power ceiling on top of slew limiting (using voltage-adjusted ceiling)
    if (allowed_current_ma > effective_ceiling) {
        allowed_current_ma = effective_ceiling;
    }

    // Update previous current for next frame's slew calculation
    // Use allowed_current (what we're actually outputting), not target
    previous_current_ma = allowed_current_ma;

    // If we're outputting what the pattern wants, no scaling needed
    if (allowed_current_ma >= target_current_ma) {
        // Periodic status log
        static uint32_t last_ok_log_time = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - last_ok_log_time > 5000) {  // Log every 5 seconds
            ESP_LOGI(TAG, "Power OK: %lumA (ceiling=%lumA, slew=%lumA/frame)",
                     target_current_ma, effective_ceiling, max_slew_ma);
            last_ok_log_time = now;
        }
        return 255;
    }

    // Calculate scale factor: allowed_current / target_current * 255
    uint8_t scale = (uint8_t)((allowed_current_ma * 255) / target_current_ma);

    // Log when limiting is active
    static uint32_t last_limit_log_time = 0;
    uint32_t now = esp_timer_get_time() / 1000;
    if (now - last_limit_log_time > 2000) {  // Log every 2 seconds
        ESP_LOGI(TAG, "Power limited: target=%lumA, allowed=%lumA (ceiling=%lumA), scale=%d/255",
                 target_current_ma, allowed_current_ma, effective_ceiling, scale);
        last_limit_log_time = now;
    }

    return scale;
}

void LEDStrip::setMaxPowerMilliamps(uint32_t ma) {
    max_power_ma = ma;
    if (ma > 0) {
        ESP_LOGI(TAG, "Power limit set to %lumA", ma);
    } else {
        ESP_LOGI(TAG, "Power limiting disabled");
    }
}

uint32_t LEDStrip::getMaxPowerMilliamps() const {
    return max_power_ma;
}

void LEDStrip::setSlewRateLimit(uint32_t ma_per_frame) {
    max_slew_ma = ma_per_frame;
    ESP_LOGI(TAG, "Slew rate limit set to %lumA/frame", ma_per_frame);
}

uint32_t LEDStrip::getSlewRateLimit() const {
    return max_slew_ma;
}

#ifdef CONFIG_BATTERY_MONITOR_ENABLED
void LEDStrip::setBatteryMonitor(BatteryMonitor* monitor) {
    battery_monitor = monitor;
    if (monitor) {
        ESP_LOGI(TAG, "Battery monitor attached - voltage-aware power limiting enabled");
    }
}
#endif

uint32_t LEDStrip::getEffectivePowerCeiling() {
    // Start with configured ceiling
    uint32_t ceiling = max_power_ma;

#ifdef CONFIG_BATTERY_MONITOR_ENABLED
    if (battery_monitor != nullptr && battery_monitor->isAvailable()) {
        // Sample voltage EVERY FRAME for fast response to voltage sag
        // ADC read is ~10-100µs, negligible vs 15ms frame time
        cached_voltage_mv = battery_monitor->readVoltage();

        uint32_t now_ms = esp_timer_get_time() / 1000;

        // Emergency shutdown logic with hysteresis
        if (emergency_shutdown) {
            // Currently in emergency - check if we can exit
            if (cached_voltage_mv >= VOLTAGE_EMERGENCY_EXIT) {
                emergency_shutdown = false;
                committed_ceiling_ma = 0;  // Reset ratchet
                ESP_LOGW(TAG, "🔋 Exiting emergency shutdown (voltage=%umV >= %umV)",
                         cached_voltage_mv, VOLTAGE_EMERGENCY_EXIT);
            } else {
                // Still in emergency - return 0 to keep LEDs off
                return 0;
            }
        } else {
            // Check if we need to enter emergency
            if (cached_voltage_mv < VOLTAGE_EMERGENCY_ENTER) {
                emergency_shutdown = true;
                ESP_LOGW(TAG, "⚠️ EMERGENCY SHUTDOWN: voltage=%umV < %umV - LEDs OFF",
                         cached_voltage_mv, VOLTAGE_EMERGENCY_ENTER);
                return 0;
            }
        }

        // Voltage-based power scaling thresholds (more aggressive)
        uint32_t voltage_ceiling = ceiling;
        if (cached_voltage_mv < VOLTAGE_VERY_LOW) {
            // Very low: 20% power
            voltage_ceiling = (ceiling * 20) / 100;
            static uint32_t last_critical_log = 0;
            if (now_ms - last_critical_log > 5000) {
                ESP_LOGW(TAG, "Very low battery: %umV - power ceiling %lumA (20%%)",
                         cached_voltage_mv, voltage_ceiling);
                last_critical_log = now_ms;
            }
        } else if (cached_voltage_mv < VOLTAGE_LOW) {
            // Low: 40% power
            voltage_ceiling = (ceiling * 40) / 100;
        } else if (cached_voltage_mv < VOLTAGE_MEDIUM_LOW) {
            // Medium-low: 60% power
            voltage_ceiling = (ceiling * 60) / 100;
        } else if (cached_voltage_mv < VOLTAGE_MEDIUM) {
            // Medium: 80% power
            voltage_ceiling = (ceiling * 80) / 100;
        }
        // >= VOLTAGE_MEDIUM (3500mV): use full configured ceiling

        // Ratchet-down logic: once we dim, stay dimmed to prevent flicker
        // Check if it's time to allow ceiling to reset (recover)
        if (committed_ceiling_ma > 0 && now_ms >= ceiling_reset_time_ms) {
            // Allow gradual recovery - only increase by small amount
            if (voltage_ceiling > committed_ceiling_ma) {
                // Voltage suggests we can go higher - allow slow recovery
                uint32_t recovery_step = max_power_ma / 20;  // 5% steps
                committed_ceiling_ma = (committed_ceiling_ma + recovery_step < voltage_ceiling)
                    ? committed_ceiling_ma + recovery_step
                    : voltage_ceiling;
                ceiling_reset_time_ms = now_ms + 2000;  // Check again in 2 seconds
            }
        }

        // If voltage ceiling is lower than committed, ratchet down immediately
        if (committed_ceiling_ma == 0 || voltage_ceiling < committed_ceiling_ma) {
            committed_ceiling_ma = voltage_ceiling;
            ceiling_reset_time_ms = now_ms + CEILING_HOLD_TIME_MS;  // Hold for 10 seconds

            if (voltage_ceiling < ceiling) {
                static uint32_t last_ratchet_log = 0;
                if (now_ms - last_ratchet_log > 2000) {
                    ESP_LOGI(TAG, "Power ceiling ratcheted: %lumA (voltage=%umV)",
                             committed_ceiling_ma, cached_voltage_mv);
                    last_ratchet_log = now_ms;
                }
            }
        }

        ceiling = committed_ceiling_ma;
    }
#endif

    return ceiling;
}

void LEDStrip::setPixelColor(uint16_t pixel, uint32_t color) {
    if (pixel < pixel_count && pixel_buffer) {
        pixel_buffer[pixel] = color;
    } else if (pixel >= pixel_count) {
        ESP_LOGW("LEDStrip", "Attempt to set pixel %d beyond strip length %d", pixel, pixel_count);
    }
}

void LEDStrip::setPixelColor(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b) {
    setPixelColor(pixel, Color(r, g, b));
}

uint32_t LEDStrip::getPixelColor(uint16_t pixel) const {
    if (pixel < pixel_count && pixel_buffer) {
        return pixel_buffer[pixel];
    }
    if (pixel >= pixel_count) {
        ESP_LOGW("LEDStrip", "Attempt to get pixel %d beyond strip length %d", pixel, pixel_count);
    }
    return 0;
}

void LEDStrip::setBrightness(uint8_t brightness) {
    brightness_level = brightness;
}

uint8_t LEDStrip::getBrightness() const {
    return brightness_level;
}

uint32_t LEDStrip::getMillis() {
    return esp_timer_get_time() / 1000;
}

void LEDStrip::clear() {
    if (pixel_buffer) {
        memset(pixel_buffer, 0, pixel_count * sizeof(uint32_t));
    }
}

void LEDStrip::clearAll(uint16_t totalPhysicalLEDs) {
    if (!rmt_channel || !led_encoder) {
        return;
    }

    // Determine buffer to use: reuse existing grb_buffer if large enough,
    // use stack for small strips, or fall back to heap allocation
    const size_t required_size = totalPhysicalLEDs * 3;
    uint8_t* clear_buffer = nullptr;
    bool needs_free = false;
    
    // Stack buffer for strips up to ~170 LEDs (512 bytes)
    uint8_t stack_buffer[512];
    
    if (grb_buffer && totalPhysicalLEDs <= pixel_count) {
        // Reuse existing GRB buffer - it's large enough
        clear_buffer = grb_buffer;
    } else if (required_size <= sizeof(stack_buffer)) {
        // Use stack buffer for small-to-medium strips
        clear_buffer = stack_buffer;
    } else {
        // Fall back to heap allocation for very long strips
        clear_buffer = new(std::nothrow) uint8_t[required_size];
        if (!clear_buffer) {
            ESP_LOGE(TAG, "Failed to allocate temp buffer for clearing %d LEDs", totalPhysicalLEDs);
            return;
        }
        needs_free = true;
    }

    // Clear the buffer (all LEDs off - 0x00 for each color component)
    memset(clear_buffer, 0, required_size);

    // Send the data to turn off all physical LEDs
    rmt_transmit_config_t clear_tx_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = 0,
        }
    };

    esp_err_t ret = rmt_transmit(rmt_channel, led_encoder, clear_buffer,
                                required_size, &clear_tx_config);
    if (ret == ESP_OK) {
        ret = rmt_tx_wait_all_done(rmt_channel, 1000); // 1 second timeout
    }

    // Only free if we allocated from heap
    if (needs_free) {
        delete[] clear_buffer;
    }

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear all physical LEDs: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Cleared all %d physical LEDs", totalPhysicalLEDs);
    }

    // Also clear our regular buffer
    clear();
}

uint16_t LEDStrip::numPixels() const {
    return pixel_count;
}

void LEDStrip::setAllColor(uint32_t color) {
    for (uint16_t i = 0; i < pixel_count; i++) {
        setPixelColor(i, color);
    }
}

void LEDStrip::setAllFade(uint8_t fade_value) {
    for (uint16_t i = 0; i < pixel_count; i++) {
        pixel_buffer[i] = ColorFade(pixel_buffer[i], fade_value);
    }
}

uint32_t LEDStrip::applyBrightness(uint32_t color) const {
    if (brightness_level == 255) {
        return color;
    }
    
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    
    // Use standard brightness scaling method (same as Adafruit_NeoPixel)
    // This approach preserves color hue better than interpolation methods
    uint16_t scale = brightness_level + 1; // +1 for efficient bit shifting
    uint8_t new_r = (r * scale) >> 8;
    uint8_t new_g = (g * scale) >> 8;
    uint8_t new_b = (b * scale) >> 8;
    
    // Debug logging for primary colors (useful for color accuracy verification)
    if (color == RED || color == GREEN || color == BLUE || color == WHITE) {
        ESP_LOGV(TAG, "Brightness: %d, Color: 0x%06lX -> RGB(%d,%d,%d) -> RGB(%d,%d,%d)",
                 brightness_level, color, r, g, b, new_r, new_g, new_b);
    }
    
    return Color(new_r, new_g, new_b);
}

// Static color utility functions
uint32_t LEDStrip::ColorFade(uint32_t color, uint8_t fade_value) {
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    
    // Use Arduino's exact fade formula: fade(0, color, fade_value)
    r = fade(0, r, fade_value);
    g = fade(0, g, fade_value);
    b = fade(0, b, fade_value);
    
    return Color(r, g, b);
}

uint32_t LEDStrip::ColorBlend(uint32_t color1, uint32_t color2, uint8_t blend_value) {
    uint8_t r1 = (color1 >> 16) & 0xFF;
    uint8_t g1 = (color1 >> 8) & 0xFF;
    uint8_t b1 = color1 & 0xFF;
    
    uint8_t r2 = (color2 >> 16) & 0xFF;
    uint8_t g2 = (color2 >> 8) & 0xFF;
    uint8_t b2 = color2 & 0xFF;
    
    uint8_t r = fade(r1, r2, blend_value);
    uint8_t g = fade(g1, g2, blend_value);
    uint8_t b = fade(b1, b2, blend_value);
    
    return Color(r, g, b);
}

uint32_t LEDStrip::ColorRandom() {
    // Use Arduino's exact ColorRandom implementation with ColorWheel
    return ColorWheel(esp_random() & 0xFF);
}

uint32_t LEDStrip::ColorWheel(uint8_t wheel_pos) {
    // Standard ColorWheel: 0=Red, 85=Green, 170=Blue
    // Position 0-84:   Red decreasing, Green increasing
    // Position 85-169: Green decreasing, Blue increasing
    // Position 170-255: Blue decreasing, Red increasing
    if (wheel_pos < 85) {
        return Color(255 - wheel_pos * 3, wheel_pos * 3, 0);
    } else if (wheel_pos < 170) {
        wheel_pos -= 85;
        return Color(0, 255 - wheel_pos * 3, wheel_pos * 3);
    } else {
        wheel_pos -= 170;
        return Color(wheel_pos * 3, 0, 255 - wheel_pos * 3);
    }
}

uint32_t LEDStrip::Color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}