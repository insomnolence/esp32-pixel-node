#include "led/player.h"
#include "led/led_strip.h"
#include "led/led_config.h"
#include "led/sequence.h"
#include "led/pattern.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>

static const char* TAG = "Player";

Player::Player()
    : sequence(nullptr),
      step(0),
      stepTime(0),
      pattern(nullptr),
      patternId(0),
      lastPatternCheck(0),
      lastStripUpdate(0),
      lastCycle(0),
      speed(35) {
    updateMutex = xSemaphoreCreateMutex();
}

Player::~Player() {
    if (updateMutex) {
        vSemaphoreDelete(updateMutex);
    }
}

void Player::SetSequence(Sequence *_sequence) {
    if (xSemaphoreTake(updateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (sequence != _sequence) {
            ESP_LOGI(TAG, "Setting new sequence");
            sequence = _sequence;
            step = 0;
            stepTime = esp_timer_get_time() / 1000; // Convert to ms
            
            // Clean up old pattern with smart pointer
            pattern.reset();
            patternId = 0;
        }
        xSemaphoreGive(updateMutex);
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex for SetSequence");
    }
}

void Player::AdvanceSequence() {
    if (!sequence) return;

    if (xSemaphoreTake(updateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ESP_LOGI(TAG, "Advancing sequence manually");
        step = sequence->Advance(step, false);
        stepTime = esp_timer_get_time() / 1000;

        // Force pattern update with smart pointer
        pattern.reset();

        // Notify callback of step change (for mesh broadcast)
        notifyStepChange();

        xSemaphoreGive(updateMutex);
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex for AdvanceSequence");
    }
}

void Player::notifyStepChange() {
    if (!stepChangeCallback || !sequence) return;

    Packet packet;
    packet.command = sequence->GetCommand(step);
    packet.brightness = sequence->GetBrightness(step);
    packet.speed = sequence->GetSpeed(step);
    packet.pattern = sequence->GetPatternId(step);

    for (int i = 0; i < 3; i++) {
        packet.color[i] = sequence->GetColors(step, i);
        packet.level[i] = sequence->GetLevels(step, i);
    }

    ESP_LOGI(TAG, "Step changed to %d, notifying callback (pattern=%d)", step, packet.pattern);
    stepChangeCallback(packet);
}

bool Player::GetCommand(Packet *command) const {
    if (!sequence || !command) {
        return false;
    }
    
    // Fill command structure with current sequence step
    command->command = sequence->GetCommand(step);
    command->brightness = sequence->GetBrightness(step);
    command->speed = sequence->GetSpeed(step);
    command->pattern = sequence->GetPatternId(step);
    
    for (int i = 0; i < 3; i++) {
        command->color[i] = sequence->GetColors(step, i);
        command->level[i] = sequence->GetLevels(step, i);
    }
    
    return true;
}

bool Player::UpdatePattern(led_time_t now, ILedStrip *strip) {
    // Only check for pattern updates periodically to save CPU
    // But check frequently enough for responsiveness
    if (now - lastPatternCheck < 10) return false;
    lastPatternCheck = now;

    if (!sequence || !strip) {
        return false;
    }

    if (xSemaphoreTake(updateMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        // Don't block too long - skip this update if mutex unavailable
        // 10ms is sufficient since mutex holders complete in <1ms
        ESP_LOGW(TAG, "Mutex timeout in UpdatePattern - skipping frame");
        return false;
    }
    
    bool patternChanged = false;
    
    // Check if we need to advance to next step based on duration
    uint32_t stepDuration = sequence->GetDuration(step);
    uint32_t elapsed = now - stepTime;
    if (stepDuration > 0 && elapsed >= stepDuration) {
        int oldStep = step;
        step = sequence->Advance(step, true); // timed advance
        ESP_LOGI(TAG, "Step %d->%d: duration=%lums, elapsed=%lums, now=%lu, stepTime=%lu",
                 oldStep, step, (unsigned long)stepDuration, (unsigned long)elapsed, (unsigned long)now, (unsigned long)stepTime);
        stepTime = now;
        patternChanged = true;

        // Clean up old pattern to force recreation
        pattern.reset();

        // Notify callback of step change (for mesh broadcast)
        notifyStepChange();
    }
    
    // Create pattern if needed (Arduino-style: check pattern ID AND colors AND levels)
    uint8_t currentPatternId = sequence->GetPatternId(step);
    bool needNewPattern = !pattern || patternId != currentPatternId;
    
    // Also check if colors or levels changed (like Arduino does)
    if (!needNewPattern && pattern) {
        for (int i = 0; i < 3; i++) {
            if (sequence->GetColors(step, i) != pattern->color(i) ||
                sequence->GetLevels(step, i) != pattern->level(i)) {
                needNewPattern = true;
                ESP_LOGI(TAG, "Pattern colors/levels changed, recreating pattern %d", currentPatternId);
                break;
            }
        }
    }
    
    if (needNewPattern) {
        // Get colors and levels for logging
        uint32_t colors[3];
        uint8_t levels[3];
        for (int i = 0; i < 3; i++) {
            colors[i] = sequence->GetColors(step, i);
            levels[i] = sequence->GetLevels(step, i);
        }
        
        ESP_LOGI(TAG, "Creating new pattern %d with colors=[0x%06lX, 0x%06lX, 0x%06lX], levels=[%d, %d, %d]", 
                 currentPatternId, colors[0], colors[1], colors[2], levels[0], levels[1], levels[2]);
        
        // Use smart pointer for automatic memory management
        pattern.reset(CreatePattern(currentPatternId));
        patternId = currentPatternId;
        patternChanged = true;
        
        if (pattern) {
            // Clear strip when switching to new pattern to prevent leftover pixels
            strip->clear();
            
            // Ensure brightness is set BEFORE initializing pattern
            uint8_t brightness = sequence->GetBrightness(step);
            
            // Apply hardware-specific brightness limiting (see led_config.h)
            uint8_t safe_brightness = LED_CLAMP_BRIGHTNESS(brightness);
            
            strip->setBrightness(safe_brightness);
            ESP_LOGI(TAG, "Set strip brightness to %d for new pattern", safe_brightness);
            
            // Initialize pattern with sequence colors and levels
            pattern->Init(strip, colors, levels, 0);
        }
    }
    
    // Update strip brightness based on sequence (clamped to hardware max in led_config.h)
    uint8_t brightness = sequence->GetBrightness(step);
    uint8_t safe_brightness = LED_CLAMP_BRIGHTNESS(brightness);
    strip->setBrightness(safe_brightness);
    
    xSemaphoreGive(updateMutex);
    return patternChanged;
}

void Player::UpdateStrip(led_time_t now, ILedStrip *strip) {
    if (!pattern || !strip || !sequence) {
        return;
    }

    // Update every 20ms for smooth animations (closer to Arduino's 16ms but still safe for ESP32)
    if ((now - lastStripUpdate) >= 20) {
        // Calculate proper timing offset like Arduino version
        led_time_t duration = pattern->GetDuration(strip);
        if (duration == 0) duration = 1000; // Default to 1 second if no duration
        
        uint8_t speed = sequence->GetSpeed(step);
        if (speed == 0) speed = 100; // Default speed
        
        // Calculate offset with speed scaling like Arduino
        led_time_t offset = ((now - stepTime) * speed / 100) % duration;
        
        // Check if we need to call Loop() for patterns that need it
        led_time_t elapsed = (now - stepTime) * speed / 100;
        led_time_t currentCycle = elapsed / duration;
        
        if (currentCycle != lastCycle) {
            // New cycle - call Loop() for patterns that implement it
            pattern->Loop(strip, offset);
            lastCycle = currentCycle;
        } else {
            // Regular update within the same cycle
            pattern->Update(strip, offset);
        }
        
        // Display the updated strip
        esp_err_t result = strip->show();
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "Failed to update LED strip: %s", esp_err_to_name(result));
        }

        lastStripUpdate = now;
    }
}