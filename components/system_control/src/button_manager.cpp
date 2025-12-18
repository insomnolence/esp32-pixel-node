#include "system_control/button_manager.h"
#include "sdkconfig.h"

const char* ButtonManager::TAG = "ButtonManager";

ButtonManager::ButtonManager(SystemHardwareInterface* hardware_impl)
    : pin1(0), pin2(0), debounce_delay(0), hardware(hardware_impl) {
#ifdef CONFIG_IDF_TARGET_ESP32C3
    ESP_LOGI(TAG, "ButtonManager created for platform: ESP32C3=true");
#else
    ESP_LOGI(TAG, "ButtonManager created for platform: ESP32C3=false");
#endif
}

esp_err_t ButtonManager::init(uint8_t pin1_gpio, uint8_t pin2_gpio, uint32_t debounce_ms) {
    if (!hardware) {
        ESP_LOGE(TAG, "No SystemHardwareInterface provided!");
        return ESP_ERR_INVALID_STATE;
    }

    this->pin1 = pin1_gpio;
    this->pin2 = pin2_gpio;
    this->debounce_delay = debounce_ms;

    ESP_LOGI(TAG, "🔘 Initializing button interface (polling mode)...");
    ESP_LOGI(TAG, "Button 1: GPIO %d, Button 2: GPIO %d, Debounce: %lums",
             pin1, pin2, (unsigned long)debounce_delay);
    ESP_LOGI(TAG, "Using GPIO polling to avoid RMT timing conflicts on ESP32-C3");
    
    // Configure Button 1 GPIO using the hardware interface
    esp_err_t ret = hardware->configureGpioInput(pin1, true, false); // Pull-up enabled
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to configure Button 1 GPIO %d: %s", 
                 pin1, esp_err_to_name(ret));
        return ret;
    }
    
    // Configure Button 2 GPIO using the hardware interface
    ret = hardware->configureGpioInput(pin2, true, false); // Pull-up enabled
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to configure Button 2 GPIO %d: %s", 
                 pin2, esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize button state tracking
    button1_last_state = hardware->readGpio(pin1);
    button2_last_state = hardware->readGpio(pin2);
    
    ESP_LOGI(TAG, "✅ Button interface initialized successfully (polling mode)");
    ESP_LOGI(TAG, "   Button 1 (Sequence): GPIO %d, initial state: %s", 
             pin1, button1_last_state ? "HIGH" : "LOW");
    ESP_LOGI(TAG, "   Button 2 (Random): GPIO %d, initial state: %s", 
             pin2, button2_last_state ? "HIGH" : "LOW");
    
    return ESP_OK;
}

void ButtonManager::setEventCallback(std::function<void(ButtonEvent)> callback) {
    eventCallback = callback;
    ESP_LOGD(TAG, "Button event callback registered");
}

void ButtonManager::processEvents() {
    if (!hardware) return; // Should not happen after init
    // Poll Button 1 (active-low: pressed = false)
    bool button1_current_state = hardware->readGpio(pin1);
    if (!button1_current_state && button1_last_state) {
        // Button 1 pressed (high to low transition)
        handleButtonPress(BUTTON_1_PRESS, &button1_last_time);
    }
    button1_last_state = button1_current_state;
    
    // Poll Button 2 (active-low: pressed = false)
    bool button2_current_state = hardware->readGpio(pin2);
    if (!button2_current_state && button2_last_state) {
        // Button 2 pressed (high to low transition)
        handleButtonPress(BUTTON_2_PRESS, &button2_last_time);
    }
    button2_last_state = button2_current_state;
}

bool ButtonManager::isDebounceTimeElapsed(uint32_t last_time) {
    if (!hardware) return true; // Safety check
    uint32_t now = hardware->getMillis(); // Convert to milliseconds
    return (now - last_time) >= debounce_delay;
}

void ButtonManager::handleButtonPress(ButtonEvent event, uint32_t* last_time) {
    if (!hardware) return; // Safety check
    if (!isDebounceTimeElapsed(*last_time)) {
        ESP_LOGD(TAG, "Button press ignored - debounce active");
        return;
    }
    
    *last_time = hardware->getMillis();
    
    const char* button_name = (event == BUTTON_1_PRESS) ? "Button 1 (Sequence)" : "Button 2 (Random)";
    int gpio_pin = (event == BUTTON_1_PRESS) ? pin1 : pin2;
    
    ESP_LOGI(TAG, "🔘 %s pressed - GPIO %d", button_name, gpio_pin);
    
    if (eventCallback) {
        eventCallback(event);
    } else {
        ESP_LOGW(TAG, "No event callback registered for button press");
    }
}
