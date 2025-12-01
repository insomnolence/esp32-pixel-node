#include "system_control/esp_idf_system_hardware.h"

const char* EspIdfSystemHardware::TAG = "EspIdfSystemHardware";

EspIdfSystemHardware::EspIdfSystemHardware() {
    // Constructor
}

uint32_t EspIdfSystemHardware::getMillis() {
    return esp_timer_get_time() / 1000;
}

bool EspIdfSystemHardware::readGpio(uint8_t gpio_num) {
    return gpio_get_level((gpio_num_t)gpio_num) == 1; // Return true for HIGH
}

esp_err_t EspIdfSystemHardware::configureGpioInput(uint8_t gpio_num, bool pull_up_en, bool pull_down_en) {
    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = pull_up_en ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = pull_down_en ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE, // Polling mode for ButtonManager
    };
    return gpio_config(&io_config);
}

void EspIdfSystemHardware::delayMillis(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
