#include "led_packet_processor.h"
#include <string.h>

const char* LedPacketProcessor::TAG = "LedPacketProcessor";

LedPacketProcessor::LedPacketProcessor() {
    ESP_LOGI(TAG, "LED Packet Processor initialized");
}

LedPacketProcessor::~LedPacketProcessor() {
}

bool LedPacketProcessor::processPacket(const GenericPacket& packet) {
    ESP_LOGI(TAG, "🔥 LedPacketProcessor::processPacket called");
    if (!packet.isValid()) {
        ESP_LOGE(TAG, "🔥 Cannot process invalid packet");
        return false;
    }
    
    ESP_LOGI(TAG, "🔥 Processing LED packet (%zu bytes)", packet.getLength());
    PacketFormat format = detectPacketFormat(packet);
    
    switch (format) {
        case PacketFormat::CURRENT_PACKET_V1:
            return processCurrentPacket(packet);
            
        // Future formats can be added here:
        // case PacketFormat::FUTURE_PACKET_V2:
        //     return processFuturePacketV2(packet);
            
        case PacketFormat::UNKNOWN:
        default:
            ESP_LOGW(TAG, "Unknown packet format, size: %zu bytes", packet.getLength());
            ESP_LOG_BUFFER_HEX(TAG, packet.getData(), packet.getLength());
            return false;
    }
}

const char* LedPacketProcessor::getProcessorName() const {
    return "LedPacketProcessor";
}

bool LedPacketProcessor::canProcessPacket(const GenericPacket& packet) const {
    return detectPacketFormat(packet) != PacketFormat::UNKNOWN;
}

void LedPacketProcessor::setLedControlCallback(std::function<void(const GenericPacket&, const char*)> callback) {
    led_control_callback = callback;
}

LedPacketProcessor::PacketFormat LedPacketProcessor::detectPacketFormat(const GenericPacket& packet) const {
    // Debug: Log actual sizes for comparison
    ESP_LOGI(TAG, "Packet size: %zu bytes, Expected Packet struct size: %zu bytes", 
             packet.getLength(), sizeof(Packet));
    
    // Check if it matches current Packet structure
    if (packet.couldBePacketType<Packet>()) {
        ESP_LOGI(TAG, "✅ Packet matches current Packet struct format");
        return PacketFormat::CURRENT_PACKET_V1;
    }
    
    // TEMPORARY FIX: Also check for 19-byte packets (expected mobile app format)
    if (packet.getLength() == 19) {
        ESP_LOGI(TAG, "📱 Detected 19-byte mobile app packet - treating as current format");
        return PacketFormat::CURRENT_PACKET_V1;
    }
    
    // Future packet format detection can be added here:
    // if (packet.couldBePacketType<FuturePacketV2>()) {
    //     return PacketFormat::FUTURE_PACKET_V2;
    // }
    
    // Could also detect based on header bytes, magic numbers, etc.
    // const uint8_t* data = packet.getData();
    // if (packet.getLength() >= 4 && data[0] == 0xAB && data[1] == 0xCD) {
    //     return PacketFormat::FUTURE_PACKET_V2;
    // }
    
    return PacketFormat::UNKNOWN;
}

bool LedPacketProcessor::processCurrentPacket(const GenericPacket& packet) {
    Packet current_pkt;
    
    // Try exact size match first
    if (packet.getPacket(current_pkt)) {
        ESP_LOGI(TAG, "✅ Exact packet format match - processing current packet format (V1)");
    } 
    // Handle 19-byte packets manually (mobile app format)
    else if (packet.getLength() == 19) {
        ESP_LOGI(TAG, "📱 Manual parsing of 19-byte mobile app packet");
        const uint8_t* data = packet.getData();
        
        // Parse the packet manually (little-endian format expected)
        current_pkt.command = data[0];
        current_pkt.brightness = data[1];
        current_pkt.speed = data[2];
        current_pkt.pattern = data[3];
        
        // Parse color array (3 x uint32_t = 12 bytes)
        memcpy(&current_pkt.color[0], &data[4], 4);
        memcpy(&current_pkt.color[1], &data[8], 4);
        memcpy(&current_pkt.color[2], &data[12], 4);
        
        // Parse level array (3 x uint8_t = 3 bytes)  
        current_pkt.level[0] = data[16];
        current_pkt.level[1] = data[17];
        current_pkt.level[2] = data[18];
        
        ESP_LOGI(TAG, "🔧 Manual parsing complete");
    } 
    else {
        ESP_LOGE(TAG, "❌ Failed to extract current packet format - size mismatch");
        ESP_LOG_BUFFER_HEX(TAG, packet.getData(), packet.getLength());
        return false;
    }
    
    logCurrentPacket(current_pkt);
    
    // Notify callback that LED control should happen
    if (led_control_callback) {
        led_control_callback(packet, "Current LED Packet V1");
    }
    
    // TODO: Add actual LED strip control here
    // For now, just log the packet contents
    ESP_LOGI(TAG, "🌈 LED Strip Control: cmd=%u, brightness=%u, speed=%u, pattern=%u",
             current_pkt.command, current_pkt.brightness, current_pkt.speed, current_pkt.pattern);
    
    return true;
}

void LedPacketProcessor::logCurrentPacket(const Packet& pkt) {
    ESP_LOGI(TAG, "=== Current Packet Details ===");
    ESP_LOGI(TAG, "Command: %u", pkt.command);
    ESP_LOGI(TAG, "Brightness: %u", pkt.brightness);
    ESP_LOGI(TAG, "Speed: %u", pkt.speed);
    ESP_LOGI(TAG, "Pattern: %u", pkt.pattern);
    ESP_LOGI(TAG, "Color: R=%lu, G=%lu, B=%lu", pkt.color[0], pkt.color[1], pkt.color[2]);
    ESP_LOGI(TAG, "Level: L1=%u, L2=%u, L3=%u", pkt.level[0], pkt.level[1], pkt.level[2]);
    ESP_LOGI(TAG, "==============================");
}