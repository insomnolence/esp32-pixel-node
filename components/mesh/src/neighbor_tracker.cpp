#include "mesh/neighbor_tracker.h"

static const char* TAG = "NeighborTracker";

NeighborTracker::NeighborTracker(uint16_t node_id)
    : local_node_id(node_id)
{
    // Reserve capacity to avoid reallocations
    neighbors.reserve(MAX_NEIGHBORS);
    ESP_LOGI(TAG, "NeighborTracker initialized for Node 0x%04X (max neighbors: %zu)", 
             local_node_id, MAX_NEIGHBORS);
}

void NeighborTracker::evictWeakestNeighbor() {
    if (neighbors.empty()) return;
    
    // Find neighbor with weakest (most negative) RSSI
    auto weakest = neighbors.begin();
    for (auto it = neighbors.begin(); it != neighbors.end(); ++it) {
        if (it->rssi < weakest->rssi) {
            weakest = it;
        }
    }
    
    ESP_LOGD(TAG, "Evicting weakest neighbor %02X:%02X:%02X:%02X:%02X:%02X (RSSI: %d) to make room",
             weakest->mac[0], weakest->mac[1], weakest->mac[2],
             weakest->mac[3], weakest->mac[4], weakest->mac[5], weakest->rssi);
    
    neighbors.erase(weakest);
}

void NeighborTracker::onPacketReceived(const uint8_t* mac, int8_t rssi) {
    uint32_t now = esp_timer_get_time() / 1000;

    for (auto& n : neighbors) {
        if (memcmp(n.mac, mac, 6) == 0) {
            // Update existing neighbor with latest RSSI
            n.rssi = rssi;
            n.last_seen = now;
            ESP_LOGV(TAG, "Updated neighbor %02X:%02X:%02X:%02X:%02X:%02X, RSSI: %d", 
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], rssi);
            return;
        }
    }

    // New neighbor - check capacity first
    if (neighbors.size() >= MAX_NEIGHBORS) {
        // At capacity - evict weakest neighbor to make room
        evictWeakestNeighbor();
    }

    // Add new neighbor
    Neighbor n;
    memcpy(n.mac, mac, 6);
    n.rssi = rssi;
    n.last_seen = now;
    neighbors.push_back(n);
    ESP_LOGD(TAG, "New neighbor added %02X:%02X:%02X:%02X:%02X:%02X, RSSI: %d (Total: %zu/%zu)",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], rssi, neighbors.size(), MAX_NEIGHBORS);
}

void NeighborTracker::cleanup() {
    uint32_t now = esp_timer_get_time() / 1000;
    neighbors.erase(
        std::remove_if(neighbors.begin(), neighbors.end(),
            [now](const Neighbor& n) {
                return now - n.last_seen > NEIGHBOR_TIMEOUT_MS;
            }),
        neighbors.end()
    );
    if (!neighbors.empty()) {
        ESP_LOGV(TAG, "Neighbor cleanup complete. Active neighbors: %zu", neighbors.size());
    } else {
        ESP_LOGV(TAG, "Neighbor cleanup complete. No active neighbors.");
    }
}

uint8_t NeighborTracker::getNeighborCount() const {
    // Note: cleanup() is called periodically from checkForRootElection()
    // No need to call it here - avoids redundant work on every count request
    return neighbors.size();
}

int8_t NeighborTracker::getAverageRssi() const {
    // Note: cleanup() is called periodically from checkForRootElection()
    // No need to call it here - avoids redundant work on every RSSI request
    if (neighbors.empty()) return 0;
    int32_t sum = 0;
    for (const auto& n : neighbors) {
        sum += n.rssi;
    }
    return sum / neighbors.size();
}
