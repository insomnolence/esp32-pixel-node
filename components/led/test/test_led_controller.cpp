#include "unity.h"
#include "led/led_controller.h"
#include "mock_led_strip.h"
#include "packet/generic_packet.h"
#include "led/pattern.h"
#include "led/sequence.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Helper to wait real time on hardware
void wait_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

TEST_CASE("LEDController initializes correctly with mock strip", "[led][controller]") {
    MockLedStrip mockStrip(60);
    LEDController* controller = new LEDController(&mockStrip);

    // Begin should succeed (use synchronous mode for testing)
    esp_err_t result = controller->begin(false);
    bool initialized = controller->isInitialized();

    // Capture values before assertions
    bool is_cleared = mockStrip.is_cleared;
    int show_count = mockStrip.show_count;
    uint16_t num_pixels = mockStrip.numPixels();

    // Clean up before assertions (Unity longjmp skips destructors)
    delete controller;

    TEST_ASSERT_EQUAL(ESP_OK, result);
    TEST_ASSERT_TRUE(initialized);
    TEST_ASSERT_TRUE(is_cleared);
    TEST_ASSERT_GREATER_THAN(0, show_count); // Should have shown init pattern
    TEST_ASSERT_EQUAL(60, num_pixels);
}

TEST_CASE("LEDController processes valid packets and applies pattern", "[led][controller]") {
    MockLedStrip mockStrip(60);
    LEDController* controller = new LEDController(&mockStrip);
    controller->begin(false);  // Synchronous mode for testing
    mockStrip.show_count = 0; // Reset show count after init pattern

    // Construct a valid packet for a FixedPattern (ID 8)
    uint8_t payload[19] = {
        1,   // Command (Update/Set Pattern)
        100, // Brightness
        50,  // Speed
        (uint8_t)PATTERN_FIXED, // Pattern ID (FixedPattern = 8)
        // Colors (3 x 4 bytes) - Red, Green, Blue
        0xFF, 0x00, 0x00, 0x00, // Red
        0x00, 0xFF, 0x00, 0x00, // Green
        0x00, 0x00, 0xFF, 0x00, // Blue
        // Levels (3 bytes)
        0, 0, 0 // Not directly used by FixedPattern in this way, but part of packet
    };

    GenericPacket packet(payload, sizeof(payload));

    // Process packet (sets the sequence but doesn't render yet)
    esp_err_t result = controller->processPacket(packet);

    // Wait real time for rate limiters in Player to allow updates
    // Player::UpdatePattern rate limit is 10ms, UpdateStrip rate limit is 20ms
    wait_ms(50);
    controller->update();  // This triggers UpdatePattern which sets brightness and creates pattern

    // Wait more time to ensure UpdateStrip runs (needs 20ms since last update)
    wait_ms(50);
    controller->update();  // This should call show()

    // Capture values AFTER update() has been called (brightness is set in UpdatePattern)
    uint8_t brightness = mockStrip.getBrightness();
    int show_count = mockStrip.show_count;
    uint32_t pixel0 = mockStrip.getPixelColor(0);
    uint32_t pixel1 = mockStrip.getPixelColor(1);

    // Explicitly delete the controller before assertions
    // This is critical because Unity's TEST_ASSERT uses longjmp which skips C++ destructors
    delete controller;

    // Now run assertions (controller is already cleaned up)
    TEST_ASSERT_EQUAL(ESP_OK, result);
    TEST_ASSERT_EQUAL(100, brightness);
    TEST_ASSERT_GREATER_THAN(0, show_count);
    TEST_ASSERT_EQUAL_UINT32(LEDStrip::Color(0xFF, 0x00, 0x00), pixel0);
    TEST_ASSERT_EQUAL_UINT32(0, pixel1); // Pixel 1 should be off
}

TEST_CASE("LEDController handles invalid packets gracefully", "[led][controller]") {
    MockLedStrip mockStrip(60);
    LEDController* controller = new LEDController(&mockStrip);
    controller->begin(false);  // Synchronous mode for testing

    // Empty packet
    GenericPacket empty;
    esp_err_t empty_result = controller->processPacket(empty);

    // Wrong size packet
    uint8_t payload[5] = {1, 2, 3, 4, 5};
    GenericPacket invalid(payload, sizeof(payload));
    esp_err_t invalid_result = controller->processPacket(invalid);

    // Clean up before assertions (Unity longjmp skips destructors)
    delete controller;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, empty_result);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, invalid_result);
}

TEST_CASE("LEDController changes modes and updates strip", "[led][controller]") {
    MockLedStrip mockStrip(60);
    LEDController* controller = new LEDController(&mockStrip);
    controller->begin(false);  // Synchronous mode for testing
    mockStrip.show_count = 0; // Reset show count

    // Player has rate limiters: UpdatePattern (10ms), UpdateStrip (20ms)
    // Need to wait real time and call update() multiple times to ensure processing

    // Default is Idle mode. Wait and update to create pattern and show
    wait_ms(50);
    controller->update();  // First update - creates pattern
    wait_ms(50);
    controller->update();  // Second update - should call show()

    // Idle pattern is random, so we can't assert a specific pixel, but we can check it's not all black.
    bool not_all_black = false;
    for (uint16_t i = 0; i < mockStrip.numPixels(); ++i) {
        if (mockStrip.getPixelColor(i) != 0) {
            not_all_black = true;
            break;
        }
    }
    int show_count_idle = mockStrip.show_count;
    mockStrip.show_count = 0; // Reset

    // Switch to Alert mode (uses AlertSequence, which is a specific pattern)
    controller->setAlertMode();
    wait_ms(50);
    controller->update();  // Creates alert pattern
    wait_ms(50);
    controller->update();  // Should call show()
    int show_count_alert = mockStrip.show_count;
    mockStrip.show_count = 0; // Reset

    controller->setIdleMode();
    wait_ms(50);
    controller->update();  // Creates idle pattern
    wait_ms(50);
    controller->update();  // Should call show()
    int show_count_final = mockStrip.show_count;

    bool initialized = controller->isInitialized();

    // Clean up before assertions (Unity longjmp skips destructors)
    delete controller;

    TEST_ASSERT_TRUE(not_all_black);
    TEST_ASSERT_GREATER_THAN(0, show_count_idle); // Ensure something was shown
    TEST_ASSERT_GREATER_THAN(0, show_count_alert); // Ensure something was shown
    TEST_ASSERT_GREATER_THAN(0, show_count_final); // Ensure something was shown
    TEST_ASSERT_TRUE(initialized);
}