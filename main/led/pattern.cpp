#include "pattern.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <cstring>

Pattern::Pattern() {
    memset(m_color, 0, sizeof(m_color));
    memset(m_level, 0, sizeof(m_level));
}

void Pattern::Init(LEDStrip *strip, const uint32_t *colors, const uint8_t *levels, led_time_t offset) {
    // Store colors and levels
    memcpy(m_color, colors, sizeof(m_color));
    memcpy(m_level, levels, sizeof(m_level));
    
    // Initialize the pattern
    Loop(strip, offset);
}

// Pattern factory
Pattern *CreatePattern(uint8_t pattern) {
    switch (pattern) {
        case PATTERN_FLASH:
            return new FlashPattern();
        case PATTERN_RAINBOW:
            return new RainbowPattern();
        case PATTERN_SPARKLE:
            return new SparklePattern();
        case PATTERN_MINI_SPARKLE:
            return new MiniSparklePattern();
        case PATTERN_MINI_TWINKLE:
            return new MiniTwinklePattern();
        case PATTERN_MARCH:
            return new MarchPattern();
        case PATTERN_WIPE:
            return new WipePattern();
        case PATTERN_GRADIENT:
            return new GradientPattern();
        case PATTERN_STROBE:
            return new StrobePattern();
        case PATTERN_FIXED:
            return new FixedPattern();
        case PATTERN_CANDY_CANE:
            return new CandyCanePattern();
        default:
            return new FixedPattern(); // Default pattern
    }
}

// FlashPattern implementation
led_time_t FlashPattern::GetDuration(const LEDStrip *strip) const {
    return 4000; // 4 second cycle like Arduino
}

void FlashPattern::Update(LEDStrip *strip, led_time_t offset) {
    if (!strip) return;
    
    // Arduino-style flash pattern with complex timing and fading
    uint16_t t = offset * 300 / GetDuration(strip);
    uint16_t o = t % 100;
    uint32_t col = color(t / 100); // Cycle through colors
    
    if ((o <= 10) || (o >= 20 && o <= 30)) {
        // Full brightness flash periods
        strip->setAllColor(col);
    } else if (o > 30 && o <= 60) {
        // Fade out period
        uint8_t f = (60 - o) * 255 / 30;
        strip->setAllColor(LEDStrip::ColorFade(col, f));
    } else {
        // Off period
        strip->setAllColor(0x000000);
    }
}

// RainbowPattern implementation
led_time_t RainbowPattern::GetDuration(const LEDStrip *strip) const {
    return 2000; // 2 second cycle like Arduino
}

void RainbowPattern::Update(LEDStrip *strip, led_time_t offset) {
    if (!strip || strip->numPixels() == 0) return;
    
    uint16_t numPixels = strip->numPixels();
    led_time_t duration = GetDuration(strip);
    if (duration == 0) duration = 1; // Prevent division by zero
    
    // Arduino-style rainbow with proper time calculation
    for (uint16_t i = 0; i < numPixels; i++) {
        uint8_t t = 255 - (offset * 255 / duration);
        uint8_t p = i * 255 / numPixels;
        strip->setPixelColor(i, LEDStrip::ColorWheel((p + t) % 255));
    }
}

// SparklePattern implementation  
led_time_t SparklePattern::GetDuration(const LEDStrip *strip) const {
    return 100; // 100ms cycle like Arduino
}

void SparklePattern::Loop(LEDStrip *strip, led_time_t offset) {  
    if (!strip || strip->numPixels() == 0) return;
    
    // Arduino-style sparkle with level-based count
    strip->setAllColor(0);
    
    uint16_t numPixels = strip->numPixels();
    // Use level[0] to control sparkle density like Arduino
    int sparkleCount = fade(1, numPixels, m_level[0]);  
    if (sparkleCount < 1) sparkleCount = 1;
    
    for (int i = 0; i < sparkleCount; i++) {
        uint16_t pixel = esp_random() % numPixels;
        uint32_t col = color(esp_random() % 3); // Pick random color from the 3 available
        if (col == 0) {
            col = LEDStrip::ColorRandom(); // Use random color if color is black
        }
        strip->setPixelColor(pixel, col);
    }
}

// MiniSparklePattern implementation
void MiniSparklePattern::Update(LEDStrip *strip, led_time_t offset) {
    if (!strip || strip->numPixels() == 0) return;
    
    // Arduino-style mini sparkle: 25% duty cycle
    if (offset > GetDuration(strip) / 4) {
        strip->setAllColor(0x000000);
    }
    // If we're in the first 25% of the cycle, the sparkles from Loop() are still visible
}

// FixedPattern implementation
led_time_t FixedPattern::GetDuration(const LEDStrip *strip) const {
    return 750; // 750ms like Arduino
}

void FixedPattern::Update(LEDStrip *strip, led_time_t offset) {
    if (!strip || strip->numPixels() == 0) return;
    
    // Arduino-style fixed pattern cycling through all 3 colors
    uint16_t step = 3 * offset / GetDuration(strip);
    uint32_t col = color(step);
    
    for (uint16_t i = 0; i < strip->numPixels(); i++) {
        strip->setPixelColor(i, (i % 3 == step) ? col : 0);
    }
}

// StrobePattern implementation
led_time_t StrobePattern::GetDuration(const LEDStrip *strip) const {
    return 750; // 750ms like Arduino (4Hz at 100% speed, 10Hz at 250% speed)
}

void StrobePattern::Update(LEDStrip *strip, led_time_t offset) {
    if (!strip) return;
    
    // Arduino-style strobe cycling through all 3 colors
    led_time_t third = GetDuration(strip) / 3;
    if ((offset / third) != (m_lastOffset / third)) {
        strip->setAllColor(color(offset / third));
    } else {
        strip->setAllColor(0x000000);
    }
    m_lastOffset = offset;
}

// MarchPattern implementation
led_time_t MarchPattern::GetDuration(const LEDStrip *strip) const {
    return 1000; // 1 second cycle like Arduino
}

void MarchPattern::Update(LEDStrip *strip, led_time_t offset) {
    if (!strip || strip->numPixels() == 0) return;
    
    uint16_t numPixels = strip->numPixels();
    uint32_t duration = GetDuration(strip);
    if (duration == 0) duration = 1;
    
    // Arduino-style march pattern with level-controlled segments and fading
    uint32_t o = (m_level[0] * 3) - (offset * m_level[0] * 3 / duration);
    
    for (uint16_t i = 0; i < numPixels; i++) {
        // Fade level based on position within segment
        uint32_t e = (i + o) % m_level[0];
        if (e > (m_level[0] / 2)) {
            e = m_level[0] - e;
        }
        if (e > (m_level[0] / 4)) {
            e = e / 2;
        } else {
            e = 0;
        }
        uint8_t f = e * 255 / (m_level[0] / 2);
        
        // Color based on segment (cycles through all 3 colors)
        uint32_t c = color(((i + o) / m_level[0]) % 3);
        
        strip->setPixelColor(i, LEDStrip::ColorFade(c, f));
    }
}

// WipePattern implementation
led_time_t WipePattern::GetDuration(const LEDStrip *strip) const {
    return 3000; // 3 second cycle like Arduino
}

void WipePattern::Update(LEDStrip *strip, led_time_t offset) {
    if (!strip || strip->numPixels() == 0) return;
    
    uint16_t numPixels = strip->numPixels();
    led_time_t duration = GetDuration(strip);
    if (duration == 0) duration = 1;
    
    // Arduino-style wipe with 3-color cycling and fading
    for (uint16_t i = 0; i < numPixels; i++) {
        int d = duration;
        int t = offset * (numPixels * 3) / d;
        t = (numPixels * 3) - t; // offset due to time
        int c = (i + t) / numPixels;
        int e = (i + t) % numPixels;
        uint8_t f = e * 255 / numPixels;
        f = (f < 128) ? 0 : ((f - 128) * 2);
        strip->setPixelColor(i, LEDStrip::ColorFade(color(c), f));
    }
}

// MiniTwinklePattern implementation
MiniTwinklePattern::MiniTwinklePattern() : m_lastDim(0), m_lastLit(0) {
}

void MiniTwinklePattern::Init(LEDStrip *strip, led_time_t offset) {
    // Arduino-style initialization
    m_lastDim = m_lastLit = offset;
    Loop(strip, offset);
}

led_time_t MiniTwinklePattern::GetDuration(const LEDStrip *strip) const {
    return 1000; // 1 second like Arduino
}

void MiniTwinklePattern::Update(LEDStrip *strip, led_time_t offset) {
    if (!strip || strip->numPixels() == 0) return;
    
    led_time_t duration = GetDuration(strip);
    uint16_t numPixels = strip->numPixels();
    
    // Dim down all pixels (Arduino-style timing-based dimming)
    led_time_t dimDelta = delta(m_lastDim, offset, duration);
    int dim = 255 - (dimDelta * 255 / duration);
    if (dim < 255) {
        for (uint16_t i = 0; i < numPixels; ++i) {
            uint32_t col = strip->getPixelColor(i);
            col = LEDStrip::ColorFade(col, dim);
            strip->setPixelColor(i, col);
        }
        m_lastDim = offset;
    }
    
    // Add new pixels as needed (Arduino-style level-based count)
    long total = fade(1, numPixels, m_level[0]);
    led_time_t litDelta = delta(m_lastLit, offset, duration);
    long todo = litDelta * total / duration;
    if (todo > 0) {
        for (; todo > 0; todo--) {
            int i = esp_random() % numPixels;
            uint32_t col = color(esp_random() % 3); // Cycle through all 3 colors
            strip->setPixelColor(i, col);
        }
        m_lastLit = offset; // Only update if we lit something!
    }
}

led_time_t MiniTwinklePattern::delta(led_time_t previous, led_time_t next, led_time_t duration) {
    if (next >= previous) {
        return next - previous;
    } else {
        return (duration - previous) + next;
    }
}

// GradientPattern implementation
GradientPattern::GradientPattern() : mp1(nullptr), mp2(nullptr) {
}

GradientPattern::~GradientPattern() {
    delete[] mp1;
    delete[] mp2;
}

led_time_t GradientPattern::GetDuration(const LEDStrip *strip) const {
    return 1000; // 1 second cycle like Arduino
}

void GradientPattern::Init(LEDStrip *strip, const uint32_t *colors, const uint8_t *levels, led_time_t offset) {
    // Store colors and levels first
    memcpy(m_color, colors, sizeof(m_color));
    memcpy(m_level, levels, sizeof(m_level));
    
    ESP_LOGI("GradientPattern", "Init: colors=[0x%06lX, 0x%06lX, 0x%06lX], level[0]=%d", 
             m_color[0], m_color[1], m_color[2], m_level[0]);
    
    Init(strip, offset);
}

void GradientPattern::Init(LEDStrip *strip, led_time_t offset) {
    uint16_t numPixels = strip->numPixels();
    
    ESP_LOGI("GradientPattern", "Setting up gradient with colors=[0x%06lX, 0x%06lX, 0x%06lX], level[0]=%d", 
             m_color[0], m_color[1], m_color[2], m_level[0]);
    
    // Set up gradient exactly like Arduino version
    grad.clearSteps();
    if (m_level[0] > 6 && m_level[0] < 249) {
        // Use level[0] to control gradient spread like Arduino
        grad.addStep(0, m_color[0]);
        grad.addStep(m_level[0] / 3, m_color[1]);
        grad.addStep((m_level[0] * 2) / 3, m_color[2]);
        grad.addStep(m_level[0], m_color[0]);
        grad.addStep(m_level[0] + 1, 0x000000);  // Arduino uses 0, not 0x000000
        grad.addStep(255, 0x000000);             // Arduino uses 0, not 0x000000
        ESP_LOGI("GradientPattern", "Level-controlled gradient: steps at 0, %d, %d, %d, %d, 255", 
                 m_level[0]/3, (m_level[0]*2)/3, m_level[0], m_level[0]+1);
    } else {
        // Default gradient for patterns that don't use level control - Arduino wraps around
        grad.addStep(0, m_color[0]);
        grad.addStep(85, m_color[1]);
        grad.addStep(170, m_color[2]);
        grad.addStep(255, m_color[0]);  // Wrap around to first color like Arduino
        ESP_LOGI("GradientPattern", "Default gradient: steps at 0, 85, 170, 255");
    }
    
    // Clean up old mapping arrays first
    delete[] mp1;
    delete[] mp2;
    mp1 = nullptr;
    mp2 = nullptr;
    
    // Allocate memory for gradient maps only if setup succeeded
    if (numPixels > 0) {
        mp1 = new uint8_t[numPixels];
        mp2 = new uint8_t[numPixels];
        
        // Initialize mapping arrays exactly like Arduino
        if (mp1 && mp2) {
            for (uint16_t i = 0; i < numPixels; i++) {
                mp1[i] = mp2[i] = i;
            }
            ESP_LOGI("GradientPattern", "Initialized mapping arrays for %d pixels", numPixels);
        } else {
            ESP_LOGE("GradientPattern", "Failed to allocate mapping arrays for %d pixels", numPixels);
        }
    }
    
    Loop(strip, offset);
}

void GradientPattern::Loop(LEDStrip *strip, led_time_t offset) {
    // Create new random map like Arduino version
    if (mp1 && mp2) {
        uint16_t numPixels = strip->numPixels();
        
        // Create temporary arrays to avoid race conditions during updates
        uint8_t *temp_mp1 = new uint8_t[numPixels];
        uint8_t *temp_mp2 = new uint8_t[numPixels];
        
        if (temp_mp1 && temp_mp2) {
            // Generate new mappings in temporary arrays
            for (uint16_t i = 0; i < numPixels; i++) {
                temp_mp1[i] = mp2[i]; // Move current map to previous
                temp_mp2[i] = esp_random() % numPixels; // Generate new random map
            }
            
            // Atomically swap the arrays
            memcpy(mp1, temp_mp1, numPixels);
            memcpy(mp2, temp_mp2, numPixels);
            
            delete[] temp_mp1;
            delete[] temp_mp2;
        } else {
            // Fallback if memory allocation fails - direct update (potential race condition but better than crash)
            ESP_LOGW("GradientPattern", "Failed to allocate temporary arrays for safe mapping update");
            for (uint16_t i = 0; i < numPixels; i++) {
                mp1[i] = mp2[i]; // Move current map to previous
                mp2[i] = esp_random() % numPixels; // Generate new random map
            }
        }
    }
    Update(strip, offset);
}

void GradientPattern::Update(LEDStrip *strip, led_time_t offset) {
    if (!strip || strip->numPixels() == 0) return;
    
    uint16_t numPixels = strip->numPixels();
    
    if (mp1 && mp2) {
        // Full Arduino-style gradient with blending - exact Arduino mapping
        led_time_t duration = GetDuration(strip);
        for (uint16_t i = 0; i < numPixels; i++) {
            // Bounds checking for random pixel mapping arrays
            if (mp1[i] >= numPixels || mp2[i] >= numPixels) {
                ESP_LOGW("GradientPattern", "Invalid pixel mapping at %d: mp1[%d]=%d, mp2[%d]=%d (max=%d)", 
                         i, i, mp1[i], i, mp2[i], numPixels-1);
                continue;
            }
            
            uint32_t c1 = grad.getColor((mp1[i] * 255) / numPixels);
            uint32_t c2 = grad.getColor((mp2[i] * 255) / numPixels);
            uint8_t blend = (offset * 255) / duration;
            uint32_t blendedColor = LEDStrip::ColorBlend(c1, c2, blend);
            strip->setPixelColor(i, blendedColor);
        }
    } else {
        ESP_LOGW("GradientPattern", "No mapping arrays - using fallback gradient");
        // Fallback: simple static gradient (exact Arduino mapping)
        for (uint16_t i = 0; i < numPixels; i++) {
            uint8_t gradPos = (i * 255) / numPixels;
            uint32_t gradientColor = grad.getColor(gradPos);
            strip->setPixelColor(i, gradientColor);
        }
    }
}

// CandyCanePattern implementation
led_time_t CandyCanePattern::GetDuration(const LEDStrip *strip) const {
    return 200; // 200ms cycle like Arduino
}

void CandyCanePattern::Update(LEDStrip *strip, led_time_t offset) {
    if (!strip || strip->numPixels() == 0) return;
    
    uint16_t numPixels = strip->numPixels();
    led_time_t duration = GetDuration(strip);
    
    int c = 0;
    if (offset < (duration / 2)) {
        c = 1;
    }
    
    for (uint16_t i = 0; i < numPixels; i++) {
        uint32_t pixelColor = color(c + (i % 2)); // Alternate between colors
        strip->setPixelColor(i, pixelColor);
    }
}

// TestPattern implementation
led_time_t TestPattern::GetDuration(const LEDStrip *strip) const {
    return 3000; // 3 second cycle
}

void TestPattern::Update(LEDStrip *strip, led_time_t offset) {
    led_time_t phaseTime = GetDuration(strip) / 3;
    
    if (offset < phaseTime) {
        // Red phase
        strip->setAllColor(RED);
    } else if (offset < phaseTime * 2) {
        // Green phase
        strip->setAllColor(GREEN);
    } else {
        // Blue phase
        strip->setAllColor(BLUE);
    }
}

// DiagnosticPattern implementation
void DiagnosticPattern::Update(LEDStrip *strip, led_time_t offset) {
    strip->clear();
    
    // Show diagnostic code as number of lit pixels
    uint16_t numPixels = (m_code < strip->numPixels()) ? m_code : strip->numPixels();
    for (uint16_t i = 0; i < numPixels; i++) {
        strip->setPixelColor(i, RED);
    }
}