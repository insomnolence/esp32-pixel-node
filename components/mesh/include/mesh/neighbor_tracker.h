#pragma once

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
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
    
    // Maximum neighbors to track - configurable via Kconfig, default 32
#ifdef CONFIG_MESH_MAX_NEIGHBORS
    static constexpr size_t MAX_NEIGHBORS = CONFIG_MESH_MAX_NEIGHBORS;
#else
    static constexpr size_t MAX_NEIGHBORS = 32;
#endif

    std::vector<Neighbor> neighbors;
    uint16_t local_node_id; // For logging

    // Evict the neighbor with weakest RSSI to make room for new ones
    void evictWeakestNeighbor();

public:
    NeighborTracker(uint16_t node_id);
    void onPacketReceived(const uint8_t* mac, int8_t rssi);
    void cleanup();
    uint8_t getNeighborCount() const;
    int8_t getAverageRssi() const;
    
    // Get configured maximum neighbor count
    static constexpr size_t getMaxNeighbors() { return MAX_NEIGHBORS; }
};
