#ifndef NEIGHBOR_MANAGER_H_
#define NEIGHBOR_MANAGER_H_

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <array>
#include <cstring>
#include <functional>

// Configuration constants
#define MAX_NEIGHBORS 32                    // Maximum neighbors per node
#define NEIGHBOR_DISCOVERY_INTERVAL_MS 5000 // Discovery beacon interval
#define NEIGHBOR_TIMEOUT_MS 15000           // Neighbor timeout (3x discovery interval)
#define MIN_RSSI_THRESHOLD -85              // Minimum RSSI for viable neighbor
#define LINK_QUALITY_SAMPLES 5              // Number of RSSI samples for averaging

enum class LinkQuality : uint8_t {
    EXCELLENT = 0, // RSSI > -50 dBm
    GOOD = 1,      // RSSI > -65 dBm  
    FAIR = 2,      // RSSI > -75 dBm
    POOR = 3,      // RSSI > -85 dBm
    UNUSABLE = 4   // RSSI <= -85 dBm
};

struct NeighborInfo {
    uint8_t mac_addr[6];           // Neighbor MAC address
    uint16_t node_id;              // Neighbor's node ID
    int8_t current_rssi;           // Most recent RSSI value
    int8_t average_rssi;           // Rolling average RSSI
    LinkQuality link_quality;      // Calculated link quality
    uint32_t last_seen_ms;         // Last beacon timestamp
    uint32_t first_seen_ms;        // When neighbor was first discovered
    uint16_t beacon_count;         // Number of beacons received
    bool is_active;                // Whether neighbor is currently active
    
    // RSSI history for averaging
    int8_t rssi_samples[LINK_QUALITY_SAMPLES];
    uint8_t rssi_sample_index;
    uint8_t rssi_sample_count;
    
    NeighborInfo() {
        memset(mac_addr, 0, 6);
        node_id = 0;
        current_rssi = -100;
        average_rssi = -100;
        link_quality = LinkQuality::UNUSABLE;
        last_seen_ms = 0;
        first_seen_ms = 0;
        beacon_count = 0;
        is_active = false;
        memset(rssi_samples, -100, sizeof(rssi_samples));
        rssi_sample_index = 0;
        rssi_sample_count = 0;
    }
    
    // Comparison operators for sorting by quality
    bool operator<(const NeighborInfo& other) const {
        if (link_quality != other.link_quality) {
            return link_quality < other.link_quality; // Better quality first
        }
        return average_rssi > other.average_rssi; // Higher RSSI first
    }
};

struct NeighborDiscoveryBeacon {
    uint32_t magic;                // Magic number for validation
    uint16_t node_id;              // Sender's node ID
    uint8_t beacon_sequence;       // Sequence number for beacon ordering
    uint32_t uptime_ms;            // Sender's uptime
    uint8_t neighbor_count;        // Number of neighbors sender has
    uint32_t timestamp_ms;         // Sender's timestamp
} __attribute__((packed));

class NeighborManager {
public:
    NeighborManager();
    ~NeighborManager();
    
    // Lifecycle management
    esp_err_t init(uint16_t local_node_id);
    esp_err_t start();
    esp_err_t stop();
    
    // Neighbor discovery
    void startNeighborDiscovery();
    void processNeighborBeacon(const uint8_t* mac_addr, const uint8_t* data, int len, int8_t rssi);
    void updateNeighborTable();
    void sendDiscoveryBeacon();
    
    // Neighbor access
    const NeighborInfo* getNeighbor(const uint8_t* mac_addr) const;
    const NeighborInfo* getNeighborByNodeId(uint16_t node_id) const;
    size_t getActiveNeighborCount() const;
    size_t getAllNeighbors(std::array<NeighborInfo, MAX_NEIGHBORS>& neighbors) const;
    
    // Link quality assessment
    LinkQuality assessLinkQuality(int8_t rssi) const;
    void updateLinkQuality(NeighborInfo& neighbor, int8_t rssi);
    const char* getLinkQualityString(LinkQuality quality) const;
    
    // Callbacks
    void setNeighborUpdateCallback(std::function<void(const NeighborInfo&, bool)> callback);
    void setBeaconSendCallback(std::function<void(const uint8_t*, size_t)> callback);
    
    // Statistics
    struct NeighborStats {
        uint32_t beacons_sent = 0;
        uint32_t beacons_received = 0;
        uint32_t neighbors_discovered = 0;
        uint32_t neighbors_lost = 0;
        uint32_t last_discovery_ms = 0;
        uint8_t current_neighbor_count = 0;
        uint8_t max_neighbor_count = 0;
    };
    
    const NeighborStats& getStats() const;
    void printNeighborTable() const;
    
private:
    static const char* TAG;
    static const uint32_t DISCOVERY_BEACON_MAGIC = 0xDEADBEEF;
    
    uint16_t local_node_id_;
    bool is_running_;
    uint8_t beacon_sequence_;
    
    // Neighbor storage - bounded array instead of dynamic containers
    std::array<NeighborInfo, MAX_NEIGHBORS> neighbors_;
    size_t neighbor_count_;
    
    // Timing
    uint32_t last_discovery_beacon_ms_;
    uint32_t startup_time_ms_;
    
    // Statistics
    NeighborStats stats_;
    
    // Callbacks
    std::function<void(const NeighborInfo&, bool)> neighbor_update_callback_;
    std::function<void(const uint8_t*, size_t)> beacon_send_callback_;
    
    // Internal methods
    NeighborInfo* findOrCreateNeighbor(const uint8_t* mac_addr, uint16_t node_id);
    NeighborInfo* findNeighborByMac(const uint8_t* mac_addr);
    void removeInactiveNeighbors();
    void updateNeighborRssi(NeighborInfo& neighbor, int8_t rssi);
    void notifyNeighborUpdate(const NeighborInfo& neighbor, bool is_new);
    uint32_t getCurrentTimeMs() const;
    bool isValidBeacon(const NeighborDiscoveryBeacon& beacon) const;
    
    // Debug methods
    void logNeighborUpdate(const NeighborInfo& neighbor, bool is_new) const;
};

#endif // NEIGHBOR_MANAGER_H_