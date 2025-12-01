#include "mesh/neighbor_tracker.h"

static const char* TAG = "NeighborTracker";

NeighborTracker::NeighborTracker(uint16_t node_id)
    : local_node_id(node_id)
{
    ESP_LOGI(TAG, "NeighborTracker initialized for Node 0x%04X", local_node_id);
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

    // New neighbor
    Neighbor n;
    memcpy(n.mac, mac, 6);
    n.rssi = rssi;
    n.last_seen = now;
    neighbors.push_back(n);
    ESP_LOGD(TAG, "New neighbor added %02X:%02X:%02X:%02X:%02X:%02X, RSSI: %d (Total: %zu)",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], rssi, neighbors.size());
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

uint8_t NeighborTracker::getNeighborCount() {
    cleanup(); // Always clean up before reporting count
    return neighbors.size();
}

int8_t NeighborTracker::getAverageRssi() {
    cleanup();
    if (neighbors.empty()) return 0;
    int32_t sum = 0;
    for (const auto& n : neighbors) {
        sum += n.rssi;
    }
    return sum / neighbors.size();
}
