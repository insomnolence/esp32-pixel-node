#ifndef LED_STRIP_H_
#define LED_STRIP_H_

#include "esp_err.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include <stdint.h>

// ESP32 LED strip driver - replaces Arduino Adafruit_NeoPixel
class LEDStrip {
public:
    LEDStrip(uint16_t pixels, uint8_t pin, uint8_t type = 0);
    ~LEDStrip();

    // Core LED strip operations (compatible with Arduino Stripper interface)
    esp_err_t begin();
    esp_err_t show();
    void setPixelColor(uint16_t pixel, uint32_t color);
    void setPixelColor(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b);
    uint32_t getPixelColor(uint16_t pixel) const;
    void setBrightness(uint8_t brightness);
    uint8_t getBrightness() const;
    void clear();
    void clearAll(uint16_t totalPhysicalLEDs); // Clear entire physical strip
    uint16_t numPixels() const;

    // Convenience methods from Arduino Stripper class
    void setAllColor(uint32_t color);
    void setAllFade(uint8_t fade_value);

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
    
    // Internal helpers
    esp_err_t initRMT();
    uint32_t applyBrightness(uint32_t color) const;
    void updatePixelBuffer();
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
    return (high - low) * value / 255 + low;
}

#endif // LED_STRIP_H_