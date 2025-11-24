#include "system_control/button_manager.h"
#include "sdkconfig.h"

const char* ButtonManager::TAG = "ButtonManager";

ButtonManager::ButtonManager() {
    ESP_LOGI(TAG, "ButtonManager created for platform: ESP32C3=%s", 
             CONFIG_IDF_TARGET_ESP32C3 ? "true" : "false");
}

esp_err_t ButtonManager::init() {
    ESP_LOGI(TAG, "🔘 Initializing button interface (polling mode)...");
    ESP_LOGI(TAG, "Button 1: GPIO %d, Button 2: GPIO %d, Debounce: %dms", 
             CONFIG_BUTTON_1_GPIO, CONFIG_BUTTON_2_GPIO, CONFIG_BUTTON_DEBOUNCE_MS);
    ESP_LOGI(TAG, "Using GPIO polling to avoid RMT timing conflicts on ESP32-C3");
    
    // Configure Button 1 GPIO (no interrupts - polling only)
    gpio_config_t button1_config = {
        .pin_bit_mask = (1ULL << CONFIG_BUTTON_1_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE  // No interrupts - polling only
    };
    
    esp_err_t ret = gpio_config(&button1_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to configure Button 1 GPIO %d: %s", 
                 CONFIG_BUTTON_1_GPIO, esp_err_to_name(ret));
        return ret;
    }
    
    // Configure Button 2 GPIO (no interrupts - polling only)  
    gpio_config_t button2_config = {
        .pin_bit_mask = (1ULL << CONFIG_BUTTON_2_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE  // No interrupts - polling only
    };
    
    ret = gpio_config(&button2_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to configure Button 2 GPIO %d: %s", 
                 CONFIG_BUTTON_2_GPIO, esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize button state tracking
    button1_last_state = gpio_get_level((gpio_num_t)CONFIG_BUTTON_1_GPIO);
    button2_last_state = gpio_get_level((gpio_num_t)CONFIG_BUTTON_2_GPIO);
    
    ESP_LOGI(TAG, "✅ Button interface initialized successfully (polling mode)");
    ESP_LOGI(TAG, "   Button 1 (Sequence): GPIO %d, initial state: %s", 
             CONFIG_BUTTON_1_GPIO, button1_last_state ? "HIGH" : "LOW");
    ESP_LOGI(TAG, "   Button 2 (Random): GPIO %d, initial state: %s", 
             CONFIG_BUTTON_2_GPIO, button2_last_state ? "HIGH" : "LOW");
    
    return ESP_OK;
}

void ButtonManager::setEventCallback(std::function<void(ButtonEvent)> callback) {
    eventCallback = callback;
    ESP_LOGD(TAG, "Button event callback registered");
}

void ButtonManager::processEvents() {
    // Poll Button 1 (active-low: pressed = false)
    bool button1_current_state = gpio_get_level((gpio_num_t)CONFIG_BUTTON_1_GPIO);
    if (!button1_current_state && button1_last_state) {
        // Button 1 pressed (high to low transition)
        handleButtonPress(BUTTON_1_PRESS, &button1_last_time);
    }
    button1_last_state = button1_current_state;
    
    // Poll Button 2 (active-low: pressed = false)
    bool button2_current_state = gpio_get_level((gpio_num_t)CONFIG_BUTTON_2_GPIO);
    if (!button2_current_state && button2_last_state) {
        // Button 2 pressed (high to low transition)
        handleButtonPress(BUTTON_2_PRESS, &button2_last_time);
    }
    button2_last_state = button2_current_state;
}

bool ButtonManager::isDebounceTimeElapsed(uint32_t last_time) {
    uint32_t now = esp_timer_get_time() / 1000; // Convert to milliseconds
    return (now - last_time) >= CONFIG_BUTTON_DEBOUNCE_MS;
}

void ButtonManager::handleButtonPress(ButtonEvent event, uint32_t* last_time) {
    if (!isDebounceTimeElapsed(*last_time)) {
        ESP_LOGD(TAG, "Button press ignored - debounce active");
        return;
    }
    
    *last_time = esp_timer_get_time() / 1000;
    
    const char* button_name = (event == BUTTON_1_PRESS) ? "Button 1 (Sequence)" : "Button 2 (Random)";
    int gpio_pin = (event == BUTTON_1_PRESS) ? CONFIG_BUTTON_1_GPIO : CONFIG_BUTTON_2_GPIO;
    
    ESP_LOGI(TAG, "🔘 %s pressed - GPIO %d", button_name, gpio_pin);
    
    if (eventCallback) {
        eventCallback(event);
    } else {
        ESP_LOGW(TAG, "No event callback registered for button press");
    }
}
