#include "player.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "Player";

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
        xSemaphoreGive(updateMutex);
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex for AdvanceSequence");
    }
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

bool Player::UpdatePattern(led_time_t now, LEDStrip *strip) {
    if (!sequence || !strip) {
        return false;
    }
    
    if (xSemaphoreTake(updateMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        // Don't block - just skip this update if mutex unavailable
        return false;
    }
    
    bool patternChanged = false;
    
    // Check if we need to advance to next step based on duration
    uint32_t stepDuration = sequence->GetDuration(step);
    if (stepDuration > 0 && (now - stepTime) >= stepDuration) {
        ESP_LOGI(TAG, "Step duration expired, advancing sequence");
        step = sequence->Advance(step, true); // timed advance
        stepTime = now;
        patternChanged = true;
        
        // Clean up old pattern to force recreation
        pattern.reset();
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
        ESP_LOGI(TAG, "Creating new pattern: %d", currentPatternId);
        
        // Use smart pointer for automatic memory management
        pattern.reset(CreatePattern(currentPatternId));
        patternId = currentPatternId;
        patternChanged = true;
        
        if (pattern) {
            // Clear strip when switching to new pattern to prevent leftover pixels
            strip->clear();
            
            // Initialize pattern with sequence colors and levels
            uint32_t colors[3];
            uint8_t levels[3];
            
            for (int i = 0; i < 3; i++) {
                colors[i] = sequence->GetColors(step, i);
                levels[i] = sequence->GetLevels(step, i);
            }
            
            pattern->Init(strip, colors, levels, 0);
        }
    }
    
    // Update strip brightness based on sequence
    uint8_t brightness = sequence->GetBrightness(step);
    strip->setBrightness(brightness);
    
    xSemaphoreGive(updateMutex);
    return patternChanged;
}

void Player::UpdateStrip(led_time_t now, LEDStrip *strip) {
    if (!pattern || !strip || !sequence) {
        return;
    }
    
    // Update every 20ms for smooth animations (closer to Arduino's 16ms but still safe for ESP32)
    if ((now - lastUpdate) >= 20) {
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
        static led_time_t lastCycle = 0;
        
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
        
        lastUpdate = now;
    }
}