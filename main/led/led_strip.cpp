#include "led_strip.h"
#include "esp_log.h"
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
        ESP_LOGE(TAG, "create LED strip encoder failed: %s", esp_err_to_name(ret));
        // Clean up RMT channel on encoder creation failure
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
    
    for (uint16_t i = 0; i < pixel_count; i++) {
        uint32_t color = applyBrightness(pixel_buffer[i]);
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

void LEDStrip::clear() {
    if (pixel_buffer) {
        memset(pixel_buffer, 0, pixel_count * sizeof(uint32_t));
    }
}

void LEDStrip::clearAll(uint16_t totalPhysicalLEDs) {
    if (!pixel_buffer || !rmt_channel || !led_encoder) {
        return;
    }
    
    // Temporarily create a larger buffer to clear all physical LEDs
    uint32_t* temp_buffer = new(std::nothrow) uint32_t[totalPhysicalLEDs];
    if (!temp_buffer) {
        ESP_LOGE(TAG, "Failed to allocate temp buffer for clearing all LEDs");
        return;
    }
    
    // Clear the temp buffer (all LEDs off)
    memset(temp_buffer, 0, totalPhysicalLEDs * sizeof(uint32_t));
    
    // Send the data to turn off all physical LEDs
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = 0,
        }
    };
    
    esp_err_t ret = rmt_transmit(rmt_channel, led_encoder, temp_buffer, 
                                totalPhysicalLEDs * sizeof(uint32_t), &tx_config);
    if (ret == ESP_OK) {
        ret = rmt_tx_wait_all_done(rmt_channel, 1000); // 1 second timeout
    }
    
    delete[] temp_buffer;
    
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
    
    // Debug logging for color corruption investigation
    if (color == RED || color == GREEN || color == BLUE || color == WHITE) {
        ESP_LOGD(TAG, "Brightness: %d, Color: 0x%06lX -> RGB(%d,%d,%d) -> RGB(%d,%d,%d)", 
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
    // Use Arduino's exact ColorWheel implementation (no inversion)
    if (wheel_pos < 85) {
        return Color(wheel_pos * 3, 255 - wheel_pos * 3, 0);
    } else if (wheel_pos < 170) {
        wheel_pos -= 85;
        return Color(255 - wheel_pos * 3, 0, wheel_pos * 3);
    } else {
        wheel_pos -= 170;
        return Color(0, wheel_pos * 3, 255 - wheel_pos * 3);
    }
}

uint32_t LEDStrip::Color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}