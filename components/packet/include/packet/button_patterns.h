#pragma once

#include "packet/packet.h"
#include <stdint.h>

// Color constants (RGB format: 0xRRGGBB)
#define PATTERN_COLOR_BLACK   0x000000
#define PATTERN_COLOR_RED     0xFF0000
#define PATTERN_COLOR_GREEN   0x00FF00
#define PATTERN_COLOR_BLUE    0x0000FF
#define PATTERN_COLOR_YELLOW  0xFFFF00
#define PATTERN_COLOR_WHITE   0xFFFFFF

// Pattern IDs (must match pattern.h)
#define PATTERN_ID_FIXED        0
#define PATTERN_ID_FLASH        1
#define PATTERN_ID_STROBE       2
#define PATTERN_ID_WIPE         3
#define PATTERN_ID_MARCH        4
#define PATTERN_ID_GRADIENT     5
#define PATTERN_ID_MINI_TWINKLE 6
#define PATTERN_ID_CANDY_CANE   7
#define PATTERN_ID_MINI_SPARKLE 8

// Command values
#define BUTTON_PATTERN_COMMAND  2  // HC_PATTERN

#ifdef __cplusplus

/**
 * @brief Predefined button-triggered patterns
 *
 * These patterns are used by ButtonLogic for button-controlled LED effects.
 * They are defined as Packet constants that can be serialized and broadcast
 * over the mesh network.
 */
namespace ButtonPatterns {

// Warning pattern - Yellow themed (Button 1: Idle -> Warning)
// Uses PATTERN_GRADIENT with yellow tones
// Note: level[0] <= 6 or >= 249 gives full-strip gradient (avoids partial spread mode)
inline Packet getWarningPattern() {
    return Packet{
        .command = BUTTON_PATTERN_COMMAND,
        .brightness = 127,
        .speed = 75,
        .pattern = PATTERN_ID_GRADIENT,
        .color = {PATTERN_COLOR_YELLOW, 0xFFFC40, PATTERN_COLOR_YELLOW},
        .level = {0, 0, 0}
    };
}

// Exit pattern - Red themed (Button 1: Warning -> Exit)
// Uses PATTERN_GRADIENT with red tones
// Note: level[0] <= 6 or >= 249 gives full-strip gradient (avoids partial spread mode)
inline Packet getExitPattern() {
    return Packet{
        .command = BUTTON_PATTERN_COMMAND,
        .brightness = 127,
        .speed = 75,
        .pattern = PATTERN_ID_GRADIENT,
        .color = {PATTERN_COLOR_RED, 0xFF4040, PATTERN_COLOR_RED},
        .level = {0, 0, 0}
    };
}

// Idle pattern - Subtle red/white/green gradient
// Uses PATTERN_GRADIENT with low brightness
// Note: level[0] <= 6 or >= 249 gives full-strip gradient (avoids partial spread mode)
inline Packet getIdlePattern() {
    return Packet{
        .command = BUTTON_PATTERN_COMMAND,
        .brightness = 20,
        .speed = 35,
        .pattern = PATTERN_ID_GRADIENT,
        .color = {PATTERN_COLOR_RED, PATTERN_COLOR_WHITE, PATTERN_COLOR_GREEN},
        .level = {0, 0, 0}
    };
}

// Random patterns for Button 2 selection
// These match the patterns in SingleRandomSequence

inline Packet getRandomPattern(int index) {
    // Available random patterns
    static const Packet patterns[] = {
        // Pattern 0: RWY twinkle
        {BUTTON_PATTERN_COMMAND, 127, 160, PATTERN_ID_MINI_TWINKLE,
         {PATTERN_COLOR_RED, PATTERN_COLOR_WHITE, PATTERN_COLOR_YELLOW}, {160, 0, 0}},

        // Pattern 1: RWG twinkle
        {BUTTON_PATTERN_COMMAND, 127, 160, PATTERN_ID_MINI_TWINKLE,
         {PATTERN_COLOR_RED, PATTERN_COLOR_WHITE, PATTERN_COLOR_GREEN}, {160, 0, 0}},

        // Pattern 2: RWR subtle gradient
        {BUTTON_PATTERN_COMMAND, 127, 35, PATTERN_ID_GRADIENT,
         {PATTERN_COLOR_RED, PATTERN_COLOR_WHITE, PATTERN_COLOR_RED}, {17, 0, 0}},

        // Pattern 3: Blue smooth gradient
        {BUTTON_PATTERN_COMMAND, 127, 75, PATTERN_ID_GRADIENT,
         {PATTERN_COLOR_BLUE, 0x8080FF, PATTERN_COLOR_BLUE}, {75, 0, 0}},

        // Pattern 4: RWB twinkle
        {BUTTON_PATTERN_COMMAND, 127, 160, PATTERN_ID_MINI_TWINKLE,
         {PATTERN_COLOR_RED, PATTERN_COLOR_WHITE, PATTERN_COLOR_BLUE}, {160, 0, 0}},

        // Pattern 5: RWG candy cane
        {BUTTON_PATTERN_COMMAND, 63, 65, PATTERN_ID_CANDY_CANE,
         {PATTERN_COLOR_RED, PATTERN_COLOR_WHITE, PATTERN_COLOR_GREEN}, {255, 0, 0}},

        // Pattern 6: RWR candy cane
        {BUTTON_PATTERN_COMMAND, 63, 100, PATTERN_ID_CANDY_CANE,
         {PATTERN_COLOR_RED, PATTERN_COLOR_WHITE, PATTERN_COLOR_RED}, {255, 0, 0}},

        // Pattern 7: RWG fixed (tree)
        {BUTTON_PATTERN_COMMAND, 127, 100, PATTERN_ID_FIXED,
         {PATTERN_COLOR_RED, PATTERN_COLOR_WHITE, PATTERN_COLOR_GREEN}, {255, 0, 0}},

        // Pattern 8: RWG march
        {BUTTON_PATTERN_COMMAND, 127, 127, PATTERN_ID_MARCH,
         {PATTERN_COLOR_RED, PATTERN_COLOR_WHITE, PATTERN_COLOR_GREEN}, {8, 0, 0}},

        // Pattern 9: RWG wipe
        {BUTTON_PATTERN_COMMAND, 127, 127, PATTERN_ID_WIPE,
         {PATTERN_COLOR_RED, PATTERN_COLOR_WHITE, PATTERN_COLOR_GREEN}, {8, 0, 0}},

        // Pattern 10: RWG sparkle
        {BUTTON_PATTERN_COMMAND, 127, 255, PATTERN_ID_MINI_SPARKLE,
         {PATTERN_COLOR_RED, PATTERN_COLOR_WHITE, PATTERN_COLOR_GREEN}, {9, 0, 0}},
    };

    const int patternCount = sizeof(patterns) / sizeof(patterns[0]);
    return patterns[index % patternCount];
}

inline int getRandomPatternCount() {
    return 11;  // Number of patterns in getRandomPattern
}

} // namespace ButtonPatterns

#endif // __cplusplus
