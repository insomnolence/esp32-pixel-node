#pragma once

#include "esp_log.h"
#include "esp_timer.h"
#include <vector>
#include <string.h>
#include <algorithm>

struct Neighbor {
    uint8_t mac[6];
    int8_t rssi;
    uint32_t last_seen; // Timestamp in ms
};

class NeighborTracker {
private:
    static constexpr uint32_t NEIGHBOR_TIMEOUT_MS = 30000; // 30 seconds (heartbeats every 10-12s)
    std::vector<Neighbor> neighbors;
    uint16_t local_node_id; // For logging

public:
    NeighborTracker(uint16_t node_id);
    void onPacketReceived(const uint8_t* mac, int8_t rssi);
    void cleanup();
    uint8_t getNeighborCount();
    int8_t getAverageRssi();
};
