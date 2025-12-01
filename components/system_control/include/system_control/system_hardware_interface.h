#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <functional>

// Forward declarations for types that might be used in callbacks or method signatures
// if a more complex FreeRTOS abstraction is needed later.
// For ButtonManager, simple types are sufficient.

/**
 * @brief Abstract interface for system-level hardware interactions.
 * 
 * This decouples system control logic from specific ESP-IDF hardware calls,
 * enabling unit testing with mocks.
 */
class SystemHardwareInterface {
public:
    virtual ~SystemHardwareInterface() = default;

    // Time
    virtual uint32_t getMillis() = 0; // Replaces esp_timer_get_time() / 1000

    // GPIO
    // Note: GPIO numbers (uint8_t) are platform-specific, but the interface can accept them.
    virtual bool readGpio(uint8_t gpio_num) = 0; // Returns true if HIGH, false if LOW
    
    // GPIO Configuration
    virtual esp_err_t configureGpioInput(uint8_t gpio_num, bool pull_up_en, bool pull_down_en) = 0;
    
    // Basic FreeRTOS / Task Delay (for debouncing)
    virtual void delayMillis(uint32_t ms) = 0; // Replaces vTaskDelay(pdMS_TO_TICKS(ms))

    // TODO: Add more methods as other system_control components are refactored
    // e.g., for NVS (nvs_flash_init, nvs_get_u8), Semaphores, Task creation.
};
