#pragma once

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "packet/generic_packet.h"
#include <functional>
#include <memory>
#include <set>
#include <vector>
#include <ctime>
#include <map>

#include "mesh/mesh_protocol.h"
#include "mesh/mesh_hardware_interface.h"

// Forward declarations
class NeighborTracker;

enum class NodeRole {
    MESH_CLIENT,             // Normal mesh node
    MESH_ROOT_ACTIVE,        // Current mesh root with BLE
    MESH_ROOT_AUTONOMOUS     // Autonomous root without BLE
};

// Global structs for NetworkStats and NodeInfo
struct NetworkStats {
    uint32_t packets_sent = 0;
    uint32_t packets_received = 0;
    uint32_t packets_dropped = 0;
    uint32_t send_failures = 0;
    uint32_t last_activity_ms = 0;

    // Simplified stats for root reporting
    uint32_t total_nodes = 0;
    int8_t avg_network_rssi = 0;
};

struct NodeInfo {
    uint8_t node_id[6];
    uint8_t neighbor_count;
    int8_t rssi; // Average RSSI to neighbors of that node
    uint32_t last_seen; // Timestamp in ms
};

class BoundedPacketTracker {
public:
    BoundedPacketTracker();
    ~BoundedPacketTracker();

    // Returns true if this is a NEW packet (not seen before), false if duplicate.
    // Automatically marks the packet as seen on first encounter.
    bool isNewPacket(uint32_t packet_id);

    // Mark a packet as seen without checking (used when sending our own packets)
    void markPacketSeen(uint32_t packet_id);

    void cleanup();

private:
    static const size_t PACKET_HISTORY_SIZE = 128; // Power of 2 for fast modulo
    static const uint32_t CLEANUP_INTERVAL_MS = 30000; // 30 seconds

    uint32_t packet_history[PACKET_HISTORY_SIZE];
    size_t history_index;
    size_t history_count;
    uint32_t last_cleanup;
    SemaphoreHandle_t tracker_mutex; 
};

class ESPNowMeshCoordinator {
public:
    ESPNowMeshCoordinator(MeshHardwareInterface* hardware);
    ~ESPNowMeshCoordinator();

    esp_err_t init();
    esp_err_t start();
    esp_err_t stop();
    
    // Role management
    NodeRole getCurrentRole() const;
    bool isRootNode() const;
    bool isBleConnected() const { return ble_connected; }
    
    // BLE integration
    void onBleConnected();
    void onBleDisconnected();
    
    // Mesh communication
    esp_err_t sendGenericPacket(const GenericPacket& packet);
    esp_err_t sendLEDPattern(const GenericPacket& pattern);
    
    // Callbacks
    void setPacketCallback(std::function<void(const GenericPacket&)> callback);
    void setRoleChangeCallback(std::function<void(NodeRole, NodeRole)> callback);
    
    // Network info
    uint16_t getNodeId() const;
    const char* getRoleString() const;
    
    // Autonomous root election (Simplified)
    void checkForRootElection();
    
    // Legacy support (to be removed/refactored)
    bool shouldAcceptBleConnection() const;
    
    // Network health monitoring
    const NetworkStats& getNetworkStats() const;

    // Node info management
    void cleanupNodeInfo();
    void updateNodeInfo(const HeartbeatPacket& hb);

    // Neighbor Management (Simplified)
    size_t getActiveNeighborCount() const;
    size_t getReachableNodeCount() const;
    int8_t getAverageNeighborRSSI() const;

    // Election helpers
    void broadcastRootClaim(RootClaimReason reason);
    bool shouldYieldTo(const RootClaimPacket& claim);
    void updateCurrentRoot(const uint8_t* root_mac);

    // Network-wide BLE status (for root election redesign)
    bool networkHasBleRoot() const;
    bool hasActiveRoot() const;

    // BLE displacement handling (dual BLE scenario)
    void setBleDisplacementCallback(std::function<void()> callback);

    // Network management
    esp_err_t initWiFi();
    esp_err_t initESPNow();
    esp_err_t transitionToRole(NodeRole new_role);
    void randomBackoff();

    // Heartbeat
    void sendHeartbeat();

private:
    static const char* TAG;
    static ESPNowMeshCoordinator* instance;
    static SemaphoreHandle_t instance_mutex;
    
    NodeRole current_role;
    uint16_t node_id;
    bool ble_connected;
    RootClaimReason current_claim_reason; // Track why we are root
    uint32_t packet_counter;
    uint8_t local_mac[6];
    
    TaskHandle_t mesh_task_handle;
    BoundedPacketTracker packet_tracker;
    NetworkStats network_stats;
    
    // Autonomous root election
    uint32_t election_timer;
    uint32_t last_root_announcement;
    bool heard_from_root;
    
    // Timestamps
    uint32_t ble_connection_uptime_ms;
    bool has_ble_connection_timestamp;
    uint32_t autonomous_root_timestamp;
    uint8_t current_root_mac[6]; // To store the MAC of the current root

    // Network-wide BLE tracking (root election redesign)
    bool network_has_ble_root;      // True if ANY node reports BLE connection
    uint8_t ble_root_mac[6];        // MAC of the node with BLE (if any)
    uint32_t last_ble_root_seen;    // Timestamp of last BLE heartbeat
    uint32_t displaced_until;       // Cooldown timestamp after being displaced by another BLE

    std::function<void(const GenericPacket&)> packet_callback;
    std::function<void(NodeRole, NodeRole)> role_change_callback;
    std::function<void()> ble_displacement_callback;
    
    // Neighbor Tracker (Simplified)
    std::unique_ptr<NeighborTracker> neighbor_tracker;

    // Network stats for Root
    std::map<uint64_t, NodeInfo> known_nodes_info;
    uint32_t last_node_cleanup;
    static constexpr uint32_t NODE_TIMEOUT_MS = 30000; // 3 missed heartbeats
    
    // Heartbeat timing
    uint32_t last_heartbeat_sent;
    uint32_t heartbeat_interval_ms; // Default 10 seconds

    // Hardware Interface callbacks
    static void onESPNowReceivedWrapper(const uint8_t *mac_addr, const uint8_t *data, int len, int8_t rssi);
    
    // Internal packet handling
    void handleReceivedPacket(const uint8_t *mac_addr, const uint8_t *data, int len, int8_t rssi);
    void forwardPacket(const void* packet, size_t len);
    esp_err_t broadcastPacket(const void* data, size_t len);
    uint32_t generatePacketId();

    // BLE displacement handling (internal)
    void handleBleDisplacement(const RootClaimPacket& claim);

    MeshHardwareInterface* hardware;
};
