#ifndef PLAYER_H_
#define PLAYER_H_

#include <stdint.h>
#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// LED timing type to avoid conflicts with system time_t  
typedef uint32_t led_time_t;

#include "../packet/packet.h"  // For RadioPixel protocol compatibility
#include "pattern.h"
#include "sequence.h"
#include "led_strip.h"

class Player {
public:
    Player()
        : sequence(nullptr), step(0), stepTime(0),
          pattern(nullptr), patternId(PATTERN_GRADIENT), 
          lastUpdate(0), lastCycle(0), speed(35), updateMutex(nullptr)
    {
        updateMutex = xSemaphoreCreateMutex();
    }
    
    ~Player() {
        if (updateMutex) {
            vSemaphoreDelete(updateMutex);
        }
    }

    //! Returns the current sequence
    const Sequence *GetSequence() const { return sequence; }
    Sequence *GetSequence() { return sequence; }

    //! Replace sequence
    void SetSequence(Sequence *_sequence);

    //! Advance the sequence via a button press
    void AdvanceSequence();

    //! Returns the current command (for mesh transmission)
    bool GetCommand(Packet *command) const;

    //! Update to the next pattern in the sequence if needed
    // Returns true if pattern changed, ie need to transmit
    bool UpdatePattern(led_time_t now, LEDStrip *strip);

    //! Update the strip with the current pattern if needed
    void UpdateStrip(led_time_t now, LEDStrip *strip);

protected:
    Sequence *sequence;
    int step; // the current step index
    led_time_t stepTime; // time we started the current step
    
    std::unique_ptr<Pattern> pattern; // Smart pointer for automatic cleanup
    uint8_t patternId;    
    led_time_t lastUpdate;
    led_time_t lastCycle;  // Track pattern cycles for thread-safe Loop() calls
    uint8_t speed;
    
    SemaphoreHandle_t updateMutex; // Mutex for thread-safe updates
};

const led_time_t FRAME_MS = 16;  // 16ms per frame (60 FPS) - smooth for LEDs

#endif // PLAYER_H_