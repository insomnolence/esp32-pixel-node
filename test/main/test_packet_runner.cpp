#include "unity.h"
#include "led/led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "ColorTest"
#define TEST_LED_PIN 7
#define TEST_LED_COUNT 10

// Visual test to verify ColorWheel output on actual hardware
static void run_colorwheel_visual_test() {
    ESP_LOGI(TAG, "=== VISUAL COLOR TEST ===");
    ESP_LOGI(TAG, "Creating LED strip on GPIO %d with %d LEDs", TEST_LED_PIN, TEST_LED_COUNT);

    LEDStrip strip(TEST_LED_COUNT, TEST_LED_PIN);
    if (strip.begin() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED strip!");
        return;
    }

    strip.setBrightness(50);  // Low brightness for safety
    strip.clear();

    // Show ColorWheel values on first 3 LEDs
    uint32_t color0 = LEDStrip::ColorWheel(0);
    uint32_t color85 = LEDStrip::ColorWheel(85);
    uint32_t color170 = LEDStrip::ColorWheel(170);

    ESP_LOGI(TAG, "ColorWheel(0)   = 0x%06lX (should be RED   0xFF0000)", color0);
    ESP_LOGI(TAG, "ColorWheel(85)  = 0x%06lX (should be GREEN 0x00FF00)", color85);
    ESP_LOGI(TAG, "ColorWheel(170) = 0x%06lX (should be BLUE  0x0000FF)", color170);

    // Set pixels: LED 0 = ColorWheel(0), LED 1 = ColorWheel(85), LED 2 = ColorWheel(170)
    strip.setPixelColor(0, color0);
    strip.setPixelColor(1, color85);
    strip.setPixelColor(2, color170);

    // Also show pure R, G, B for comparison on LEDs 4, 5, 6
    strip.setPixelColor(4, 0xFF0000);  // Pure RED
    strip.setPixelColor(5, 0x00FF00);  // Pure GREEN
    strip.setPixelColor(6, 0x0000FF);  // Pure BLUE

    ESP_LOGI(TAG, "LED 0-2: ColorWheel(0, 85, 170)");
    ESP_LOGI(TAG, "LED 4-6: Pure RED, GREEN, BLUE for comparison");
    ESP_LOGI(TAG, "If ColorWheel is correct, LED 0 should match LED 4, etc.");

    strip.show();

    ESP_LOGI(TAG, "Visual test displayed for 5 seconds...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    strip.clear();
    strip.show();
    ESP_LOGI(TAG, "=== VISUAL TEST COMPLETE ===");
}

extern "C" void app_main(void)
{
    // Run visual color test first
    run_colorwheel_visual_test();

    // Then run unit tests
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
