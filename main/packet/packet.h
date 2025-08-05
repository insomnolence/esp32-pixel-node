#ifndef PACKET_H_
#define PACKET_H_

#include <stdint.h>

class Packet {
public:
    Packet();
    ~Packet();

    uint8_t command;
    uint8_t brightness;
    uint8_t speed;
    uint8_t pattern;
    uint32_t color[3];
    uint8_t level[3];
};

#endif // PACKET_H_
