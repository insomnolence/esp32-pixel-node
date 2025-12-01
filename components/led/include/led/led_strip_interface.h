#pragma once

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Abstract interface for LED Strip hardware drivers.
 * 
 * Allows decoupling LED control logic (animations, sequences) from the specific
 * hardware implementation (RMT, SPI, Bit-banging, or external library).
 */
class ILedStrip {
public:
    virtual ~ILedStrip() = default;

    // Lifecycle
    virtual esp_err_t begin() = 0;
    
    // Core operations
    virtual esp_err_t show() = 0;
    virtual void setPixelColor(uint16_t pixel, uint32_t color) = 0;
    virtual void setPixelColor(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b) = 0;
    virtual uint32_t getPixelColor(uint16_t pixel) const = 0;
    
    // Configuration
    virtual void setBrightness(uint8_t brightness) = 0;
    virtual uint8_t getBrightness() const = 0;
    
    // Utilities
    virtual void clear() = 0;
    virtual void clearAll(uint16_t totalPhysicalLEDs) = 0;
    virtual uint16_t numPixels() const = 0;
    
    // Bulk operations (optional but useful for efficiency)
    virtual void setAllColor(uint32_t color) = 0;

    // Time provider for LED logic
    virtual uint32_t getMillis() = 0;
};
