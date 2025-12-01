#pragma once

#include "system_control/system_hardware_interface.h"
#include <map>
#include <vector>

class MockSystemHardware : public SystemHardwareInterface {
public:
    MockSystemHardware() : current_millis(0) {}
    virtual ~MockSystemHardware() = default;

    // State
    uint32_t current_millis;
    std::map<uint8_t, bool> gpio_states; // gpio_num -> is_high
    std::vector<uint32_t> delays_ms;
    std::map<uint8_t, bool> gpio_configured_as_input;
    std::map<uint8_t, bool> gpio_pull_up_enabled;
    std::map<uint8_t, bool> gpio_pull_down_enabled;

    // Interface Implementation
    uint32_t getMillis() override { return current_millis; }

    bool readGpio(uint8_t gpio_num) override {
        auto it = gpio_states.find(gpio_num);
        if (it != gpio_states.end()) {
            return it->second; // Return the mocked state
        }
        return false; // Default to LOW if not set
    }

    esp_err_t configureGpioInput(uint8_t gpio_num, bool pull_up_en, bool pull_down_en) override {
        gpio_configured_as_input[gpio_num] = true;
        gpio_pull_up_enabled[gpio_num] = pull_up_en;
        gpio_pull_down_enabled[gpio_num] = pull_down_en;
        return ESP_OK;
    }

    void delayMillis(uint32_t ms) override {
        delays_ms.push_back(ms);
        // In a real test, you might advance current_millis here by 'ms'
        // But for ButtonManager, processEvents will control time advancement.
    }

    // Helper methods for tests
    void setGpioState(uint8_t gpio_num, bool is_high) {
        gpio_states[gpio_num] = is_high;
    }

    void advanceTime(uint32_t ms) {
        current_millis += ms;
    }
};
