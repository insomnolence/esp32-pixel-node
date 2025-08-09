#include "neighbor_manager.h"
#include "esp_mac.h"
#include <algorithm>

const char* NeighborManager::TAG = "NeighborManager";

NeighborManager::NeighborManager() 
    : local_node_id_(0)
    , is_running_(false)
    , beacon_sequence_(0)
    , neighbor_count_(0)
    , last_discovery_beacon_ms_(0)
    , startup_time_ms_(0)
    , neighbor_update_callback_(nullptr)
{
    // Initialize neighbor array
    neighbors_.fill(NeighborInfo{});
}

NeighborManager::~NeighborManager() {
    stop();
}

esp_err_t NeighborManager::init(uint16_t local_node_id) {
    local_node_id_ = local_node_id;
    startup_time_ms_ = getCurrentTimeMs();
    
    ESP_LOGI(TAG, "Initialized NeighborManager for node %u", local_node_id_);
    return ESP_OK;
}

esp_err_t NeighborManager::start() {
    if (is_running_) {
        ESP_LOGW(TAG, "NeighborManager already running");
        return ESP_OK;
    }
    
    is_running_ = true;
    last_discovery_beacon_ms_ = getCurrentTimeMs();
    
    ESP_LOGI(TAG, "Started neighbor discovery");
    return ESP_OK;
}

esp_err_t NeighborManager::stop() {
    if (!is_running_) {
        return ESP_OK;
    }
    
    is_running_ = false;
    ESP_LOGI(TAG, "Stopped neighbor discovery");
    return ESP_OK;
}

void NeighborManager::startNeighborDiscovery() {
    if (!is_running_) {
        return;
    }
    
    uint32_t current_time = getCurrentTimeMs();
    
    // Send discovery beacon periodically
    if (current_time - last_discovery_beacon_ms_ >= NEIGHBOR_DISCOVERY_INTERVAL_MS) {
        sendDiscoveryBeacon();
        last_discovery_beacon_ms_ = current_time;
    }
    
    // Update neighbor table and remove inactive neighbors
    updateNeighborTable();
}

void NeighborManager::sendDiscoveryBeacon() {
    NeighborDiscoveryBeacon beacon{};
    beacon.magic = DISCOVERY_BEACON_MAGIC;
    beacon.node_id = local_node_id_;
    beacon.beacon_sequence = beacon_sequence_++;
    beacon.uptime_ms = getCurrentTimeMs() - startup_time_ms_;
    beacon.neighbor_count = static_cast<uint8_t>(getActiveNeighborCount());
    beacon.timestamp_ms = getCurrentTimeMs();
    
    // Send beacon through mesh coordinator if callback is set
    if (beacon_send_callback_) {
        beacon_send_callback_(reinterpret_cast<const uint8_t*>(&beacon), sizeof(beacon));
        ESP_LOGD(TAG, "📡 Discovery beacon sent: seq=%u, neighbors=%u", 
                 beacon.beacon_sequence, beacon.neighbor_count);
    } else {
        ESP_LOGW(TAG, "❌ No beacon send callback configured - beacon not transmitted");
    }
    
    stats_.beacons_sent++;
    stats_.last_discovery_ms = getCurrentTimeMs();
}

void NeighborManager::processNeighborBeacon(const uint8_t* mac_addr, const uint8_t* data, int len, int8_t rssi) {
    if (len < sizeof(NeighborDiscoveryBeacon)) {
        ESP_LOGW(TAG, "Received malformed beacon (len=%d, expected=%d)", 
                 len, sizeof(NeighborDiscoveryBeacon));
        return;
    }
    
    const NeighborDiscoveryBeacon* beacon = reinterpret_cast<const NeighborDiscoveryBeacon*>(data);
    
    // Validate beacon
    if (!isValidBeacon(*beacon)) {
        ESP_LOGW(TAG, "Received invalid beacon from " MACSTR, MAC2STR(mac_addr));
        return;
    }
    
    // Ignore our own beacons
    if (beacon->node_id == local_node_id_) {
        return;
    }
    
    // Find or create neighbor entry
    NeighborInfo* neighbor = findOrCreateNeighbor(mac_addr, beacon->node_id);
    if (!neighbor) {
        ESP_LOGW(TAG, "Cannot add neighbor - table full");
        return;
    }
    
    // Update neighbor information
    bool is_new_neighbor = !neighbor->is_active;
    neighbor->is_active = true;
    neighbor->last_seen_ms = getCurrentTimeMs();
    neighbor->beacon_count++;
    
    if (is_new_neighbor) {
        neighbor->first_seen_ms = neighbor->last_seen_ms;
        stats_.neighbors_discovered++;
    }
    
    // Update RSSI and link quality
    updateNeighborRssi(*neighbor, rssi);
    
    // Notify callback if registered
    notifyNeighborUpdate(*neighbor, is_new_neighbor);
    
    stats_.beacons_received++;
    
    ESP_LOGV(TAG, "Updated neighbor %u (" MACSTR ") RSSI=%d, quality=%s", 
             neighbor->node_id, MAC2STR(neighbor->mac_addr), 
             rssi, getLinkQualityString(neighbor->link_quality));
}

void NeighborManager::updateNeighborTable() {
    uint32_t current_time = getCurrentTimeMs();
    size_t active_neighbors = 0;
    
    for (size_t i = 0; i < neighbor_count_; ++i) {
        NeighborInfo& neighbor = neighbors_[i];
        
        if (!neighbor.is_active) {
            continue;
        }
        
        // Check if neighbor has timed out
        if (current_time - neighbor.last_seen_ms > NEIGHBOR_TIMEOUT_MS) {
            ESP_LOGI(TAG, "Neighbor %u (" MACSTR ") timed out", 
                     neighbor.node_id, MAC2STR(neighbor.mac_addr));
            
            neighbor.is_active = false;
            stats_.neighbors_lost++;
            
            // Notify callback of lost neighbor
            if (neighbor_update_callback_) {
                neighbor_update_callback_(neighbor, false);
            }
        } else {
            active_neighbors++;
        }
    }
    
    stats_.current_neighbor_count = static_cast<uint8_t>(active_neighbors);
    if (active_neighbors > stats_.max_neighbor_count) {
        stats_.max_neighbor_count = static_cast<uint8_t>(active_neighbors);
    }
    
    // Compact the array by moving inactive neighbors to the end
    removeInactiveNeighbors();
}

const NeighborInfo* NeighborManager::getNeighbor(const uint8_t* mac_addr) const {
    for (size_t i = 0; i < neighbor_count_; ++i) {
        if (neighbors_[i].is_active && 
            memcmp(neighbors_[i].mac_addr, mac_addr, 6) == 0) {
            return &neighbors_[i];
        }
    }
    return nullptr;
}

const NeighborInfo* NeighborManager::getNeighborByNodeId(uint16_t node_id) const {
    for (size_t i = 0; i < neighbor_count_; ++i) {
        if (neighbors_[i].is_active && neighbors_[i].node_id == node_id) {
            return &neighbors_[i];
        }
    }
    return nullptr;
}

size_t NeighborManager::getActiveNeighborCount() const {
    size_t count = 0;
    for (size_t i = 0; i < neighbor_count_; ++i) {
        if (neighbors_[i].is_active) {
            count++;
        }
    }
    return count;
}

size_t NeighborManager::getAllNeighbors(std::array<NeighborInfo, MAX_NEIGHBORS>& neighbors) const {
    size_t active_count = 0;
    
    for (size_t i = 0; i < neighbor_count_ && active_count < MAX_NEIGHBORS; ++i) {
        if (neighbors_[i].is_active) {
            neighbors[active_count] = neighbors_[i];
            active_count++;
        }
    }
    
    // Sort by link quality (best first)
    std::sort(neighbors.begin(), neighbors.begin() + active_count);
    
    return active_count;
}

LinkQuality NeighborManager::assessLinkQuality(int8_t rssi) const {
    if (rssi > -50) return LinkQuality::EXCELLENT;
    if (rssi > -65) return LinkQuality::GOOD;
    if (rssi > -75) return LinkQuality::FAIR;
    if (rssi > -85) return LinkQuality::POOR;
    return LinkQuality::UNUSABLE;
}

void NeighborManager::updateLinkQuality(NeighborInfo& neighbor, int8_t rssi) {
    neighbor.current_rssi = rssi;
    
    // Update RSSI samples for averaging
    neighbor.rssi_samples[neighbor.rssi_sample_index] = rssi;
    neighbor.rssi_sample_index = (neighbor.rssi_sample_index + 1) % LINK_QUALITY_SAMPLES;
    
    if (neighbor.rssi_sample_count < LINK_QUALITY_SAMPLES) {
        neighbor.rssi_sample_count++;
    }
    
    // Calculate average RSSI
    int32_t rssi_sum = 0;
    for (uint8_t i = 0; i < neighbor.rssi_sample_count; ++i) {
        rssi_sum += neighbor.rssi_samples[i];
    }
    neighbor.average_rssi = static_cast<int8_t>(rssi_sum / neighbor.rssi_sample_count);
    
    // Update link quality based on average RSSI
    neighbor.link_quality = assessLinkQuality(neighbor.average_rssi);
}

const char* NeighborManager::getLinkQualityString(LinkQuality quality) const {
    switch (quality) {
        case LinkQuality::EXCELLENT: return "EXCELLENT";
        case LinkQuality::GOOD: return "GOOD";
        case LinkQuality::FAIR: return "FAIR";
        case LinkQuality::POOR: return "POOR";
        case LinkQuality::UNUSABLE: return "UNUSABLE";
        default: return "UNKNOWN";
    }
}

void NeighborManager::setNeighborUpdateCallback(std::function<void(const NeighborInfo&, bool)> callback) {
    neighbor_update_callback_ = callback;
}

void NeighborManager::setBeaconSendCallback(std::function<void(const uint8_t*, size_t)> callback) {
    beacon_send_callback_ = callback;
}

const NeighborManager::NeighborStats& NeighborManager::getStats() const {
    return stats_;
}

void NeighborManager::printNeighborTable() const {
    ESP_LOGI(TAG, "=== Neighbor Table (Active: %zu) ===", getActiveNeighborCount());
    
    for (size_t i = 0; i < neighbor_count_; ++i) {
        const NeighborInfo& neighbor = neighbors_[i];
        if (!neighbor.is_active) continue;
        
        uint32_t age_ms = getCurrentTimeMs() - neighbor.first_seen_ms;
        ESP_LOGI(TAG, "  Node %u: " MACSTR " RSSI=%d/%d %s beacons=%u age=%lums", 
                 neighbor.node_id, MAC2STR(neighbor.mac_addr),
                 neighbor.current_rssi, neighbor.average_rssi,
                 getLinkQualityString(neighbor.link_quality),
                 neighbor.beacon_count, age_ms);
    }
    
    ESP_LOGI(TAG, "Stats: sent=%lu, received=%lu, discovered=%lu, lost=%lu",
             stats_.beacons_sent, stats_.beacons_received, 
             stats_.neighbors_discovered, stats_.neighbors_lost);
}

// Private methods

NeighborInfo* NeighborManager::findOrCreateNeighbor(const uint8_t* mac_addr, uint16_t node_id) {
    // First, try to find existing neighbor
    NeighborInfo* existing = findNeighborByMac(mac_addr);
    if (existing) {
        return existing;
    }
    
    // Try to find inactive slot to reuse
    for (size_t i = 0; i < neighbor_count_; ++i) {
        if (!neighbors_[i].is_active) {
            memcpy(neighbors_[i].mac_addr, mac_addr, 6);
            neighbors_[i].node_id = node_id;
            return &neighbors_[i];
        }
    }
    
    // Create new entry if space available
    if (neighbor_count_ < MAX_NEIGHBORS) {
        memcpy(neighbors_[neighbor_count_].mac_addr, mac_addr, 6);
        neighbors_[neighbor_count_].node_id = node_id;
        return &neighbors_[neighbor_count_++];
    }
    
    return nullptr; // Table full
}

NeighborInfo* NeighborManager::findNeighborByMac(const uint8_t* mac_addr) {
    for (size_t i = 0; i < neighbor_count_; ++i) {
        if (memcmp(neighbors_[i].mac_addr, mac_addr, 6) == 0) {
            return &neighbors_[i];
        }
    }
    return nullptr;
}

void NeighborManager::removeInactiveNeighbors() {
    size_t write_index = 0;
    
    for (size_t read_index = 0; read_index < neighbor_count_; ++read_index) {
        if (neighbors_[read_index].is_active) {
            if (read_index != write_index) {
                neighbors_[write_index] = neighbors_[read_index];
            }
            write_index++;
        }
    }
    
    // Clear unused entries
    for (size_t i = write_index; i < neighbor_count_; ++i) {
        neighbors_[i] = NeighborInfo{};
    }
    
    neighbor_count_ = write_index;
}

void NeighborManager::updateNeighborRssi(NeighborInfo& neighbor, int8_t rssi) {
    updateLinkQuality(neighbor, rssi);
    logNeighborUpdate(neighbor, false);
}

void NeighborManager::notifyNeighborUpdate(const NeighborInfo& neighbor, bool is_new) {
    if (neighbor_update_callback_) {
        neighbor_update_callback_(neighbor, is_new);
    }
    
    if (is_new) {
        ESP_LOGI(TAG, "New neighbor: Node %u (" MACSTR ") RSSI=%d %s", 
                 neighbor.node_id, MAC2STR(neighbor.mac_addr),
                 neighbor.current_rssi, getLinkQualityString(neighbor.link_quality));
    }
}

uint32_t NeighborManager::getCurrentTimeMs() const {
    return esp_timer_get_time() / 1000;
}

bool NeighborManager::isValidBeacon(const NeighborDiscoveryBeacon& beacon) const {
    return beacon.magic == DISCOVERY_BEACON_MAGIC && 
           beacon.node_id != 0 && 
           beacon.neighbor_count <= MAX_NEIGHBORS;
}

void NeighborManager::logNeighborUpdate(const NeighborInfo& neighbor, bool is_new) const {
    ESP_LOGD(TAG, "%s neighbor %u: RSSI %d->%d, quality=%s", 
             is_new ? "New" : "Updated",
             neighbor.node_id, neighbor.current_rssi, neighbor.average_rssi,
             getLinkQualityString(neighbor.link_quality));
}