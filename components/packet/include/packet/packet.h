#pragma once

#include <stdint.h>
#include <stddef.h>

// Wire format size for LED packets (matches Flutter app format)
#define LED_PACKET_WIRE_SIZE 19

// Convert to C-compatible struct for ESP32-C3 compatibility
#pragma pack(push, 1)
typedef struct {
    uint8_t command;
    uint8_t brightness;
    uint8_t speed;
    uint8_t pattern;
    uint32_t color[3];  // RGB format: 0xRRGGBB
    uint8_t level[3];
} Packet;
#pragma pack(pop)

#ifdef __cplusplus
/**
 * @brief Serialize a Packet to wire format (19 bytes, little-endian colors)
 * @param packet Source packet with RGB colors (0xRRGGBB format)
 * @param buffer Output buffer (must be at least LED_PACKET_WIRE_SIZE bytes)
 * @param bufferSize Size of output buffer
 * @return Number of bytes written, or 0 on error
 */
inline size_t serializePacketToWire(const Packet& packet, uint8_t* buffer, size_t bufferSize) {
    if (!buffer || bufferSize < LED_PACKET_WIRE_SIZE) {
        return 0;
    }

    buffer[0] = packet.command;
    buffer[1] = packet.brightness;
    buffer[2] = packet.speed;
    buffer[3] = packet.pattern;

    // Serialize colors to little-endian format {B, G, R, 0} as expected by the protocol
    for (int i = 0; i < 3; i++) {
        uint32_t color = packet.color[i];
        uint8_t* colorBytes = &buffer[4 + i * 4];
        colorBytes[0] = (color >> 0) & 0xFF;   // B -> byte 0
        colorBytes[1] = (color >> 8) & 0xFF;   // G -> byte 1
        colorBytes[2] = (color >> 16) & 0xFF;  // R -> byte 2
        colorBytes[3] = 0;                      // A (unused) -> byte 3
    }

    buffer[16] = packet.level[0];
    buffer[17] = packet.level[1];
    buffer[18] = packet.level[2];

    return LED_PACKET_WIRE_SIZE;
}
#endif
