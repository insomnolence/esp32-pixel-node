#ifndef LED_PACKET_PROCESSOR_H_
#define LED_PACKET_PROCESSOR_H_

#include "generic_packet.h"
#include "packet.h" // Current packet format
#include "esp_log.h"
#include <functional>

// LED packet processor that handles current and future packet formats
class LedPacketProcessor : public PacketProcessor {
public:
    LedPacketProcessor();
    ~LedPacketProcessor() override;
    
    // PacketProcessor interface
    bool processPacket(const GenericPacket& packet) override;
    const char* getProcessorName() const override;
    bool canProcessPacket(const GenericPacket& packet) const override;
    
    // Set callback for when LED commands are processed
    void setLedControlCallback(std::function<void(const GenericPacket&, const char*)> callback);

private:
    static const char* TAG;
    std::function<void(const GenericPacket&, const char*)> led_control_callback;
    
    // Current packet format handlers
    bool processCurrentPacket(const GenericPacket& packet);
    void logCurrentPacket(const Packet& pkt);
    
    // Packet format detection
    enum class PacketFormat {
        UNKNOWN,
        CURRENT_PACKET_V1,
        // FUTURE_PACKET_V2,
        // FUTURE_PACKET_V3
    };
    
    PacketFormat detectPacketFormat(const GenericPacket& packet) const;
};

#endif // LED_PACKET_PROCESSOR_H_