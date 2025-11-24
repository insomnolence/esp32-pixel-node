#include "unity.h"
#include "led/gradient.h"
#include "led/pattern.h"
#include "led/led_strip.h"

static const uint32_t COLOR_RED = LEDStrip::Color(255, 0, 0);
static const uint32_t COLOR_GREEN = LEDStrip::Color(0, 255, 0);
static const uint32_t COLOR_BLUE = LEDStrip::Color(0, 0, 255);

TEST_CASE("Gradient handles boundaries and interpolation", "[led][gradient]") {
    Gradient grad;

    // No steps -> returns 0
    TEST_ASSERT_EQUAL_UINT32(0, grad.getColor(100));

    // Simple two-step gradient
    grad.addStep(0, COLOR_RED);
    grad.addStep(255, COLOR_BLUE);

    TEST_ASSERT_EQUAL_UINT32(COLOR_RED, grad.getColor(0));
    TEST_ASSERT_EQUAL_UINT32(COLOR_BLUE, grad.getColor(255));

    // Midpoint should be blend of red and blue
    uint32_t expected_mid = LEDStrip::ColorBlend(COLOR_RED, COLOR_BLUE, 128);
    TEST_ASSERT_EQUAL_UINT32(expected_mid, grad.getColor(128));
}

TEST_CASE("LEDStrip color utilities blend and fade as expected", "[led][color]") {
    uint32_t half_red = LEDStrip::ColorFade(COLOR_RED, 128);
    TEST_ASSERT_EQUAL_UINT32(LEDStrip::Color(127, 0, 0), half_red);

    uint32_t blend = LEDStrip::ColorBlend(COLOR_RED, COLOR_BLUE, 128);
    TEST_ASSERT_EQUAL_UINT32(LEDStrip::Color(127, 0, 127), blend);

    TEST_ASSERT_EQUAL_UINT32(COLOR_RED, LEDStrip::ColorWheel(0));
    TEST_ASSERT_EQUAL_UINT32(COLOR_GREEN, LEDStrip::ColorWheel(85));
    TEST_ASSERT_EQUAL_UINT32(COLOR_BLUE, LEDStrip::ColorWheel(170));
}

TEST_CASE("FixedPattern lights every third pixel", "[led][pattern]") {
    LEDStrip strip(6, 18);
    FixedPattern pattern;
    uint32_t colors[3] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE};
    uint8_t levels[3] = {0, 0, 0};

    pattern.Init(&strip, colors, levels, 0);

    // After init (offset 0) only every 3rd pixel starting at 0 should be lit red
    for (uint16_t i = 0; i < strip.numPixels(); ++i) {
        uint32_t expected = (i % 3 == 0) ? COLOR_RED : 0;
        TEST_ASSERT_EQUAL_UINT32(expected, strip.getPixelColor(i));
    }

    led_time_t offset = pattern.GetDuration(&strip) / 3;
    pattern.Update(&strip, offset);
    for (uint16_t i = 0; i < strip.numPixels(); ++i) {
        uint32_t expected = (i % 3 == 1) ? COLOR_GREEN : 0;
        TEST_ASSERT_EQUAL_UINT32(expected, strip.getPixelColor(i));
    }
}

TEST_CASE("StrobePattern alternates between color segments and black", "[led][pattern]") {
    LEDStrip strip(4, 18);
    StrobePattern pattern;
    uint32_t colors[3] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE};
    uint8_t levels[3] = {0, 0, 0};
    pattern.Init(&strip, colors, levels, 0);

    led_time_t duration = pattern.GetDuration(&strip);
    led_time_t third = duration / 3;

    // Initial update (offset 0) remains off (per implementation)
    pattern.Update(&strip, 0);
    for (uint16_t i = 0; i < strip.numPixels(); ++i) {
        TEST_ASSERT_EQUAL_UINT32(0, strip.getPixelColor(i));
    }

    // Enter next segment -> should light with second color
    pattern.Update(&strip, third);
    for (uint16_t i = 0; i < strip.numPixels(); ++i) {
        TEST_ASSERT_EQUAL_UINT32(COLOR_GREEN, strip.getPixelColor(i));
    }

    // Still within same segment -> should go dark again
    pattern.Update(&strip, third + third / 2);
    for (uint16_t i = 0; i < strip.numPixels(); ++i) {
        TEST_ASSERT_EQUAL_UINT32(0, strip.getPixelColor(i));
    }

    // Advance to final segment -> should use blue
    pattern.Update(&strip, 2 * third);
    for (uint16_t i = 0; i < strip.numPixels(); ++i) {
        TEST_ASSERT_EQUAL_UINT32(COLOR_BLUE, strip.getPixelColor(i));
    }
}
