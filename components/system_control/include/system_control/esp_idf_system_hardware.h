#pragma once

#include "system_control/system_hardware_interface.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/**
 * @brief Concrete implementation of SystemHardwareInterface using ESP-IDF APIs.
 */
class EspIdfSystemHardware : public SystemHardwareInterface {
public:
    EspIdfSystemHardware();
    virtual ~EspIdfSystemHardware() = default;

    uint32_t getMillis() override;
    bool readGpio(uint8_t gpio_num) override;
    esp_err_t configureGpioInput(uint8_t gpio_num, bool pull_up_en, bool pull_down_en) override;
    void delayMillis(uint32_t ms) override;

private:
    static const char* TAG;
};
