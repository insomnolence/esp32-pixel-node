#pragma once

#include "led/led_strip.h"
#include "led/player.h"
#include "led/sequence.h"
#include "packet/generic_packet.h"
#include "packet/packet.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <memory>

// Default LED pin based on target platform
#ifdef CONFIG_IDF_TARGET_ESP32C3
#define DEFAULT_LED_PIN 7   // ESP32C3 board
#define DEFAULT_LED_COUNT 60  // ESP32C3: Reduced count to prevent power-related resets
#elif CONFIG_IDF_TARGET_ESP32
#define DEFAULT_LED_PIN 12  // ESP32 board  
#define DEFAULT_LED_COUNT 144 // ESP32: Full strip
#else
#define DEFAULT_LED_PIN 7   // Default fallback
#define DEFAULT_LED_COUNT 144 // Default: Full strip
#endif
#define PHYSICAL_LED_STRIP_LENGTH 144  // Total LEDs on physical strip (for clearing extras)

// TEMPORARY SOLUTION: Hardware power safety limits for ESP32-C3
// TODO: Replace with dynamic power management based on actual power requirements analysis
// FIXME: Investigate if component selection (TPS61322A boost converter) allows higher brightness
#ifdef CONFIG_IDF_TARGET_ESP32C3
#define HARDWARE_MAX_BRIGHTNESS_ESP32C3 110  // TPS61322A safe limit (~1.6A max output)
#endif

// Dual-core processing message types
enum class LEDCommandType {
    UPDATE_PATTERN,
    SET_SEQUENCE,
    BUTTON_FEEDBACK,  // Route button feedback through LED task to prevent RMT conflicts
    SHUTDOWN,
    ADVANCE_SEQUENCE  // Route sequence advancement through LED task to prevent race conditions
};

// Include feedback types
#include "common/button_feedback_types.h"

// Message structure for LED processing queue
struct LEDCommand {
    LEDCommandType type;
    union {
        struct {
            led_time_t timestamp;
        } update;
        struct {
            Sequence* sequence;
        } setSeq;
        struct {
            FeedbackType feedback_type;
            uint32_t duration_ms;
        } buttonFeedback;
    } data;
};

// Centralized LED controller that manages strip, player, and sequences
class LEDController {
public:
    LEDController(uint8_t pin = DEFAULT_LED_PIN, uint16_t count = DEFAULT_LED_COUNT);
    ~LEDController();

    // Initialization
    esp_err_t begin();
    
    // Process packets from BLE/mesh
    esp_err_t processPacket(const GenericPacket& packet);
    
    // Manual sequence control
    void setIdleMode();
    void setAlertMode();
    void setRandomMode();
    void setWarningMode();  // Warning pattern (yellow)
    void setExitMode();     // Exit pattern (red)
    void setRandomModeWithNewPattern(); // Random mode with fresh random pattern selection
    
    // Note: Visual feedback methods moved to ButtonFeedbackController for local-only feedback
    void advanceSequence();
    
    // Update loop (call from main loop)
    void update();
    
    // Status
    bool isInitialized() const { return initialized; }
    const char* getCurrentSequenceType() const;
    
    // Button feedback through LED task (prevents RMT timing conflicts on ESP32-C3)
    esp_err_t showButtonFeedback(FeedbackType type, uint32_t duration_ms);
    
    // Direct LED access for local feedback (ButtonFeedbackController) - DEPRECATED: Use showButtonFeedback instead
    LEDStrip* getLEDStrip() { return strip; }
    
private:
    LEDStrip* strip;
    Player* player;
    
    // Pre-defined sequences
    IdleSequence* idleSequence;
    AlertSequence* alertSequence;
    RandomSequence* randomSequence;  // Keep for compatibility
    SingleRandomSequence* singleRandomSequence;  // Button 2: single pattern then idle
    PacketSequence* packetSequence;
    
    // Parameterized sequences  
    ParameterizedSequence* warningSequence;
    ParameterizedSequence* exitSequence;
    
    // Current packet for PacketSequence
    Packet currentPacket;
    
    bool initialized;
    const uint8_t ledPin;      // Const - never changes after construction
    const uint16_t ledCount;   // Const - never changes after construction
    
    // Button 2 single random pattern tracking
    bool singleRandomActive;
    uint32_t singleRandomStartTime;
    
    // LED task processing members (both ESP32 and ESP32C3)
    QueueHandle_t ledCommandQueue;
    TaskHandle_t ledTaskHandle;
    bool ledTaskMode;
    
    static const char* TAG;
    
    // Helper methods
    esp_err_t parsePacketData(const GenericPacket& packet, Packet& parsedPacket) const;
    void logPacketInfo(const Packet& pkt) const;
    void cleanup(); // Clean up allocated resources on failure
    void setSequence(Sequence* sequence); // Platform-adaptive sequence setting
    
    // LED task processing methods (both ESP32 and ESP32C3)
    esp_err_t initLEDTask();
    void shutdownLEDTask();
    static void ledProcessingTaskWrapper(void* parameter);
    void ledProcessingTask();
    bool sendLEDCommand(const LEDCommand& command, TickType_t timeout = portMAX_DELAY);
    
    // Button feedback handling in LED task context
    void handleButtonFeedbackInLEDTask(FeedbackType type, uint32_t duration_ms);
};