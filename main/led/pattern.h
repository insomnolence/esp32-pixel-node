#ifndef PATTERN_H_
#define PATTERN_H_

#include <stdint.h>

// LED timing type to avoid conflicts with system time_t
typedef uint32_t led_time_t;

#include "led_strip.h"
#include "gradient.h"

// Forward declarations
class Gradient;

class Pattern {
public:
    Pattern();
    virtual ~Pattern() {}
    
    // Returns loop duration, time offset never goes above this
    virtual led_time_t GetDuration(const LEDStrip *strip) const { return 40; }

    // Assume nothing, setup all pixels
    virtual void Init(LEDStrip *strip, const uint32_t *colors, const uint8_t *levels, led_time_t offset);

    // Assume nothing, setup all pixels
    virtual void Init(LEDStrip *strip, led_time_t offset) { Loop(strip, offset); }

    // Restarting after a loop expired, but not first call 
    virtual void Loop(LEDStrip *strip, led_time_t offset) { Update(strip, offset); }

    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) { }

    // Returns color
    uint32_t color(int index) const {
        return m_color[index % 3];
    }

    // Returns level
    uint8_t level(int index) const {
        return m_level[index % 3];
    }

protected:
    uint32_t m_color[3];
    uint8_t m_level[3];
};

// Pattern factory
Pattern *CreatePattern(uint8_t pattern);

// Flash the entire strip
class FlashPattern : public Pattern {
public:
    // Returns loop duration, time offset never goes above this
    virtual led_time_t GetDuration(const LEDStrip *strip) const override;

    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) override;
};

// Rainbow!
class RainbowPattern : public Pattern {
public:
    // Returns loop duration, time offset never goes above this
    virtual led_time_t GetDuration(const LEDStrip *strip) const override;

    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) override;
};

class SparklePattern : public Pattern {
public:
    // Returns loop duration, time offset never goes above this
    virtual led_time_t GetDuration(const LEDStrip *strip) const override;

    // Update pixels as needed
    virtual void Loop(LEDStrip *strip, led_time_t offset) override;
};

class MiniSparklePattern : public SparklePattern {
public:
    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) override;
};

class MiniTwinklePattern : public Pattern {
public:
    MiniTwinklePattern();
    
    // Assume nothing, setup all pixels
    virtual void Init(LEDStrip *strip, led_time_t offset) override;

    // Returns loop duration, time offset never goes above this
    virtual led_time_t GetDuration(const LEDStrip *strip) const override;
    
    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) override;

protected:
    led_time_t delta(led_time_t previous, led_time_t next, led_time_t duration);

    led_time_t m_lastDim;
    led_time_t m_lastLit;
};

class MarchPattern : public Pattern {
public:
    // Returns loop duration, time offset never goes above this
    virtual led_time_t GetDuration(const LEDStrip *strip) const override;
    
    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) override;
};

class WipePattern : public Pattern {
public:
    // Returns loop duration, time offset never goes above this
    virtual led_time_t GetDuration(const LEDStrip *strip) const override;

    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) override;
};

class GradientPattern : public Pattern {
public:
    GradientPattern();
    ~GradientPattern();
    
    // Returns loop duration, time offset never goes above this
    virtual led_time_t GetDuration(const LEDStrip *strip) const override;

    // Assume nothing, setup all pixels
    virtual void Init(LEDStrip *strip, const uint32_t *colors, const uint8_t *levels, led_time_t offset) override;
    virtual void Init(LEDStrip *strip, led_time_t offset) override;

    // Restarting after a loop expired, but not first call 
    virtual void Loop(LEDStrip *strip, led_time_t offset) override;
    
    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) override;
        
private:
    Gradient grad;
    uint8_t *mp1, *mp2;
};

// Strobe the entire strip
class StrobePattern : public Pattern {
public:
    StrobePattern() : m_lastOffset(0) {}
    
    // Returns loop duration, time offset never goes above this
    virtual led_time_t GetDuration(const LEDStrip *strip) const override;

    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) override;

    led_time_t m_lastOffset;
};

class FixedPattern : public Pattern {
public:
    // Returns loop duration, time offset never goes above this
    virtual led_time_t GetDuration(const LEDStrip *strip) const override;

    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) override;
};

class CandyCanePattern : public Pattern {
public:
    // Returns loop duration, time offset never goes above this
    virtual led_time_t GetDuration(const LEDStrip *strip) const override;

    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) override;
};

class TestPattern : public Pattern {
public:
    // Returns loop duration, time offset never goes above this
    virtual led_time_t GetDuration(const LEDStrip *strip) const override;

    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) override;
};

class DiagnosticPattern : public Pattern {
public:
    DiagnosticPattern(int code = 0) : m_code(code) { }

    // Update pixels as needed
    virtual void Update(LEDStrip *strip, led_time_t offset) override;

    int m_code;
};

// Pattern IDs matching Arduino RadioPixel protocol
enum PatternID {
    PATTERN_MINI_TWINKLE = 0,
    PATTERN_MINI_SPARKLE = 1,
    PATTERN_SPARKLE = 2,
    PATTERN_RAINBOW = 3,
    PATTERN_FLASH = 4,
    PATTERN_MARCH = 5,
    PATTERN_WIPE = 6,
    PATTERN_GRADIENT = 7,
    PATTERN_FIXED = 8,
    PATTERN_STROBE = 9,
    PATTERN_CANDY_CANE = 10
};

#endif // PATTERN_H_