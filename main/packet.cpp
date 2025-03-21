#include "packet.h"

Packet::Packet() : command(0), brightness(0), speed(0), pattern(0) {
    // Constructor
    for (int i = 0; i < 3; ++i) {
        color[i] = 0;
        level[i] = 0;
    }
}

Packet::~Packet() {
    // Destructor
}
