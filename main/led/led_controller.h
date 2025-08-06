#ifndef LED_CONTROLLER_H_
#define LED_CONTROLLER_H_

#include "led_strip.h"
#include "player.h"
#include "sequence.h"
#include "../packet/generic_packet.h"
#include "../packet/packet.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <memory>

// Default LED pin based on target platform
#ifdef CONFIG_IDF_TARGET_ESP32C3
#define DEFAULT_LED_PIN 7   // ESP32C3 board
#elif CONFIG_IDF_TARGET_ESP32
#define DEFAULT_LED_PIN 12  // ESP32 board  
#else
#define DEFAULT_LED_PIN 7   // Default fallback
#endif
#define DEFAULT_LED_COUNT 144
#define PHYSICAL_LED_STRIP_LENGTH 144  // Total LEDs on physical strip (for clearing extras)

// Dual-core processing message types
enum class LEDCommandType {
    UPDATE_PATTERN,
    SET_SEQUENCE,
    SHUTDOWN
};

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
    void advanceSequence();
    
    // Update loop (call from main loop)
    void update();
    
    // Status
    bool isInitialized() const { return initialized; }
    const char* getCurrentSequenceType() const;
    
private:
    LEDStrip* strip;
    Player* player;
    
    // Pre-defined sequences
    IdleSequence* idleSequence;
    AlertSequence* alertSequence;
    RandomSequence* randomSequence;
    PacketSequence* packetSequence;
    
    // Current packet for PacketSequence
    Packet currentPacket;
    
    bool initialized;
    const uint8_t ledPin;      // Const - never changes after construction
    const uint16_t ledCount;   // Const - never changes after construction
    
    // Dual-core processing members (ESP32 only)
#ifdef CONFIG_IDF_TARGET_ESP32
    QueueHandle_t ledCommandQueue;
    TaskHandle_t ledTaskHandle;
    bool dualCoreMode;
#endif
    
    static const char* TAG;
    
    // Helper methods
    esp_err_t parsePacketData(const GenericPacket& packet, Packet& parsedPacket) const;
    void logPacketInfo(const Packet& pkt) const;
    void cleanup(); // Clean up allocated resources on failure
    void setSequence(Sequence* sequence); // Platform-adaptive sequence setting
    
    // Dual-core processing methods (ESP32 only)
#ifdef CONFIG_IDF_TARGET_ESP32
    esp_err_t initDualCore();
    void shutdownDualCore();
    static void ledProcessingTaskWrapper(void* parameter);
    void ledProcessingTask();
    bool sendLEDCommand(const LEDCommand& command, TickType_t timeout = portMAX_DELAY);
#endif
};

#endif // LED_CONTROLLER_H_