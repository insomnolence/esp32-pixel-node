#ifndef LED_CONTROLLER_H_
#define LED_CONTROLLER_H_

#include "led_strip.h"
#include "player.h"
#include "sequence.h"
#include "../packet/generic_packet.h"
#include "../packet/packet.h"
#include "esp_timer.h"
#include <memory>

#define DEFAULT_LED_PIN 7 // Dan's board
//#define DEFAULT_LED_PIN 13 // Adafruit board
#define DEFAULT_LED_COUNT 60
#define PHYSICAL_LED_STRIP_LENGTH 144  // Total LEDs on physical strip (for clearing extras)

// Centralized LED controller that manages strip, player, and sequences
class LEDController {
public:
    LEDController(uint8_t pin = DEFAULT_LED_PIN, uint16_t count = DEFAULT_LED_COUNT);
    ~LEDController();

    // Initialization
    esp_err_t begin();
    
    // Process packets from BLE/mesh
    bool processPacket(const GenericPacket& packet);
    
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
    
    static const char* TAG;
    
    // Helper methods
    bool parsePacketData(const GenericPacket& packet, Packet& parsedPacket) const;
    void logPacketInfo(const Packet& pkt) const;
};

#endif // LED_CONTROLLER_H_