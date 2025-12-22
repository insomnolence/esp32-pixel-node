#pragma once

/**
 * @file led_config.h
 * @brief Centralized LED brightness and pattern configuration
 *
 * All brightness values and pattern-related constants should be defined here
 * for easy adjustment. This prevents scattered hardcoded values throughout
 * the codebase.
 */

#include <stdint.h>

// =============================================================================
// BRIGHTNESS CONFIGURATION
// =============================================================================

/**
 * @brief Maximum brightness allowed for any pattern
 *
 * This is the hardware-safe maximum brightness level. All patterns should
 * respect this limit to prevent power issues and ensure consistent behavior
 * across ESP32 and ESP32-C3 variants.
 */
#define LED_BRIGHTNESS_MAX          35

/**
 * @brief Brightness for idle/standby patterns
 *
 * Lower brightness for subtle ambient effects when no active pattern is running.
 */
#define LED_BRIGHTNESS_IDLE         17

/**
 * @brief Brightness for normal active patterns
 *
 * Standard brightness for regular pattern playback (random sequences,
 * user-triggered patterns, etc.)
 */
#define LED_BRIGHTNESS_NORMAL       35

/**
 * @brief Brightness for alert/attention patterns
 *
 * Used for warning, exit, and alert sequences. Capped at LED_BRIGHTNESS_MAX.
 */
#define LED_BRIGHTNESS_ALERT        35

/**
 * @brief Brightness for flash/strobe effects within alert sequences
 *
 * Even high-intensity effects are capped to the maximum safe brightness.
 */
#define LED_BRIGHTNESS_FLASH        35

/**
 * @brief Brightness for initial attention-grabbing flash (e.g., parameterized sequences)
 */
#define LED_BRIGHTNESS_ATTENTION    35

// =============================================================================
// HARDWARE-SPECIFIC OVERRIDES
// =============================================================================

/**
 * @brief Hardware-specific maximum brightness
 *
 * ESP32-C3: Capped at 35 as a conservative default. Dynamic power limiting
 *           (CONFIG_LED_MAX_POWER_MA) provides smarter scaling based on actual
 *           power draw rather than a fixed brightness cap.
 * ESP32: No hardware limit (full 0-255 range supported).
 *
 * The clamp macro only affects ESP32-C3; ESP32 can use higher brightness
 * if pattern values are increased.
 */
#ifdef CONFIG_IDF_TARGET_ESP32C3
#define LED_BRIGHTNESS_HARDWARE_MAX 35
#else
#define LED_BRIGHTNESS_HARDWARE_MAX 255  // ESP32: No hardware limit
#endif

// =============================================================================
// POWER LIMITING CONFIGURATION
// =============================================================================

/**
 * @brief Current draw per LED color channel at full brightness
 *
 * Typical WS2812B draws ~20mA per color channel (R, G, B) at full brightness.
 * Full white (RGB all 255) = 60mA per LED.
 */
#define LED_MA_PER_CHANNEL          20

/**
 * @brief Maximum current per LED at full white brightness
 *
 * 3 channels * 20mA = 60mA per LED
 */
#define LED_MA_PER_LED_FULL_WHITE   60

/**
 * @brief Default power budget from Kconfig (0 = disabled)
 *
 * ESP32-C3 custom board: 450mA (TPS61322A boost converter limit ~500mA max output)
 * ESP32/other: 0 (disabled - assumes adequate external power supply)
 */
#ifdef CONFIG_LED_MAX_POWER_MA
#define LED_DEFAULT_MAX_POWER_MA    CONFIG_LED_MAX_POWER_MA
#else
#define LED_DEFAULT_MAX_POWER_MA    0
#endif

/**
 * @brief Maximum current increase per frame (slew rate limiting)
 *
 * Limits how fast the total LED current can increase between frames.
 * This prevents voltage sag from sudden current spikes that can cause
 * brownout resets on battery-powered devices.
 *
 * At 67fps (15ms/frame), 150mA/frame allows:
 * - 0 to 450mA in 3 frames (~45ms) - imperceptible visually
 * - Candy cane swap (~165mA delta) in ~1 frame (~15ms) - instant
 *
 * Only active when power limiting is enabled (LED_DEFAULT_MAX_POWER_MA > 0).
 */
#ifdef CONFIG_LED_MAX_SLEW_MA_PER_FRAME
#define LED_MAX_SLEW_MA_PER_FRAME   CONFIG_LED_MAX_SLEW_MA_PER_FRAME
#else
#define LED_MAX_SLEW_MA_PER_FRAME   150
#endif

// =============================================================================
// PATTERN TIMING CONFIGURATION
// =============================================================================

/**
 * @brief Duration for permanent/infinite patterns (0 = no timeout)
 */
#define LED_DURATION_PERMANENT      0

/**
 * @brief Standard pattern duration in milliseconds (30 seconds)
 */
#define LED_DURATION_STANDARD       30000

/**
 * @brief Short alert flash duration in milliseconds
 */
#define LED_DURATION_FLASH_SHORT    300

/**
 * @brief Medium alert duration in milliseconds
 */
#define LED_DURATION_FLASH_MEDIUM   500

/**
 * @brief Long alert/hold duration in milliseconds
 */
#define LED_DURATION_HOLD           1000

/**
 * @brief Extended pattern duration in milliseconds (2 seconds)
 */
#define LED_DURATION_EXTENDED       2000

// =============================================================================
// SPEED PRESETS
// =============================================================================

/**
 * @brief Slow animation speed (for subtle idle patterns)
 */
#define LED_SPEED_SLOW              35

/**
 * @brief Medium animation speed
 */
#define LED_SPEED_MEDIUM            75

/**
 * @brief Fast animation speed
 */
#define LED_SPEED_FAST              100

/**
 * @brief Very fast animation speed (for strobes, etc.)
 */
#define LED_SPEED_VERY_FAST         150

// =============================================================================
// HELPER MACROS
// =============================================================================

/**
 * @brief Clamp brightness to hardware maximum
 */
#define LED_CLAMP_BRIGHTNESS(b) ((b) > LED_BRIGHTNESS_HARDWARE_MAX ? LED_BRIGHTNESS_HARDWARE_MAX : (b))
