#pragma once

#include "led/led_strip_interface.h"
#include "esp_timer.h"
#include <vector>
#include <cstring>

class MockLedStrip : public ILedStrip {
public:
    MockLedStrip(uint16_t num_pixels) : pixels(num_pixels, 0), brightness(255) {}
    virtual ~MockLedStrip() = default;

    // State
    std::vector<uint32_t> pixels;
    uint8_t brightness;
    bool is_cleared = false;
    int show_count = 0;

    // Interface
    esp_err_t begin() override { return ESP_OK; }
    
    esp_err_t show() override { 
        show_count++;
        return ESP_OK; 
    }
    
    void setPixelColor(uint16_t pixel, uint32_t color) override {
        if (pixel < pixels.size()) {
            pixels[pixel] = color;
        }
    }
    
    void setPixelColor(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b) override {
        if (pixel < pixels.size()) {
            pixels[pixel] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        }
    }
    
    uint32_t getPixelColor(uint16_t pixel) const override {
        if (pixel < pixels.size()) {
            return pixels[pixel];
        }
        return 0;
    }
    
    void setBrightness(uint8_t b) override {
        brightness = b;
    }
    
    uint8_t getBrightness() const override {
        return brightness;
    }
    
    void clear() override {
        std::fill(pixels.begin(), pixels.end(), 0);
        is_cleared = true;
    }
    
    void clearAll(uint16_t total) override {
        clear();
    }
    
    uint16_t numPixels() const override {
        return pixels.size();
    }
    
    void setAllColor(uint32_t color) override {
        std::fill(pixels.begin(), pixels.end(), color);
    }

    // Use real hardware time on hardware - mocking time doesn't make sense on-target
    uint32_t getMillis() override { return esp_timer_get_time() / 1000; }
};
