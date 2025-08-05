#ifndef PACKET_H_
#define PACKET_H_

#include <stdint.h>

// Convert to C-compatible struct for ESP32-C3 compatibility
#pragma pack(push, 1)
typedef struct {
    uint8_t command;
    uint8_t brightness;
    uint8_t speed;
    uint8_t pattern;
    uint32_t color[3];
    uint8_t level[3];
} Packet;
#pragma pack(pop)

#endif // PACKET_H_
