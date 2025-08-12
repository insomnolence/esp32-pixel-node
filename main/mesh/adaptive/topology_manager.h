#ifndef TOPOLOGY_MANAGER_H_
#define TOPOLOGY_MANAGER_H_

#include "neighbor_manager.h"
#include "esp_err.h"
#include "esp_log.h"
#include <array>
#include <functional>

// Maximum nodes in the topology map
#define MAX_TOPOLOGY_NODES 64

enum class AdaptiveNodeRole : uint8_t {
    UNKNOWN = 0,        // Role not yet determined
    EDGE = 1,           // Edge node with few neighbors
    BRIDGE = 2,         // Bridge node connecting network segments  
    HUB = 3,            // Hub node with many neighbors
    CRITICAL_BRIDGE = 4 // Bridge whose removal would partition network
};

struct TopologyNode {
    uint16_t node_id;
    uint8_t mac_addr[6];
    AdaptiveNodeRole role;
    uint8_t neighbor_count;
    int8_t best_rssi;           // Best RSSI to this node
    uint16_t hop_count;         // Hops from local node
    uint32_t last_update_ms;
    bool is_reachable;
};

struct TopologyUpdate {
    uint32_t magic;
    uint16_t sender_node_id;
    uint8_t update_sequence;
    uint8_t topology_version;
    uint8_t node_count;
    TopologyNode nodes[8]; // Limited nodes per update to fit in packet
} __attribute__((packed));

class TopologyManager {
public:
    TopologyManager();
    ~TopologyManager();
    
    // Lifecycle
    esp_err_t init(uint16_t local_node_id, NeighborManager* neighbor_manager);
    esp_err_t start();
    esp_err_t stop();
    
    // Topology management
    void updateTopology();
    void processTopologyUpdate(const uint8_t* sender_mac, const uint8_t* data, int len);
    void broadcastTopologyUpdate();
    
    // Role determination
    AdaptiveNodeRole determineLocalRole() const;
    void updateNodeRole(uint16_t node_id, AdaptiveNodeRole role);
    AdaptiveNodeRole getNodeRole(uint16_t node_id) const;
    
    // Network analysis
    bool isCriticalBridge() const;
    bool wouldPartitionNetwork(uint16_t node_id) const;
    size_t getNetworkDiameter() const;
    size_t getReachableNodeCount() const;
    
    // Topology queries
    const TopologyNode* getNode(uint16_t node_id) const;
    size_t getAllNodes(std::array<TopologyNode, MAX_TOPOLOGY_NODES>& nodes) const;
    void getNodesByRole(AdaptiveNodeRole role, std::array<uint16_t, MAX_TOPOLOGY_NODES>& node_ids, size_t& count) const;
    
    // Bridge detection
    void identifyBridgeNodes();
    void identifyCriticalBridges();
    
    // Callbacks
    void setTopologyChangeCallback(std::function<void(const TopologyNode&, bool)> callback);
    void setRoleChangeCallback(std::function<void(uint16_t, AdaptiveNodeRole, AdaptiveNodeRole)> callback);
    void setBroadcastCallback(std::function<void(const uint8_t*, size_t)> callback);
    
    // Statistics and debugging
    struct TopologyStats {
        uint32_t topology_updates_sent = 0;
        uint32_t topology_updates_received = 0;
        uint32_t role_changes = 0;
        uint32_t bridge_nodes = 0;
        uint32_t critical_bridges = 0;
        uint32_t last_full_update_ms = 0;
        uint8_t topology_version = 0;
    };
    
    const TopologyStats& getStats() const;
    void printTopologyMap() const;
    const char* getRoleString(AdaptiveNodeRole role) const;
    
private:
    static const char* TAG;
    static const uint32_t TOPOLOGY_UPDATE_MAGIC = 0xFEEDFACE;
    static const uint32_t TOPOLOGY_UPDATE_INTERVAL_MS = 10000; // 10 seconds
    static const uint32_t TOPOLOGY_NODE_TIMEOUT_MS = 30000;    // 30 seconds
    
    uint16_t local_node_id_;
    NeighborManager* neighbor_manager_;
    bool is_running_;
    uint8_t update_sequence_;
    
    // Topology state
    std::array<TopologyNode, MAX_TOPOLOGY_NODES> topology_nodes_;
    size_t node_count_;
    AdaptiveNodeRole local_role_;
    
    // Reusable buffer to avoid stack allocation
    std::array<NeighborInfo, MAX_NEIGHBORS> neighbor_buffer_;
    
    // Timing
    uint32_t last_topology_update_ms_;
    uint32_t last_role_analysis_ms_;
    
    // Statistics
    TopologyStats stats_;
    
    // Callbacks
    std::function<void(const TopologyNode&, bool)> topology_change_callback_;
    std::function<void(uint16_t, AdaptiveNodeRole, AdaptiveNodeRole)> role_change_callback_;
    std::function<void(const uint8_t*, size_t)> broadcast_callback_;
    
    // Internal methods
    TopologyNode* findOrCreateNode(uint16_t node_id);
    TopologyNode* findNode(uint16_t node_id);
    void updateNodeFromNeighbor(const NeighborInfo& neighbor);
    void removeStaleNodes();
    void analyzeNetworkTopology();
    AdaptiveNodeRole calculateNodeRole(const TopologyNode& node) const;
    void notifyTopologyChange(const TopologyNode& node, bool is_new);
    void notifyRoleChange(uint16_t node_id, AdaptiveNodeRole old_role, AdaptiveNodeRole new_role);
    uint32_t getCurrentTimeMs() const;
    bool isValidTopologyUpdate(const TopologyUpdate& update) const;
    
    // Network analysis helpers
    void buildConnectivityGraph();
    bool isNodeCriticalBridge(uint16_t node_id) const;
    size_t countConnectedComponents(uint16_t excluded_node = 0) const;
    void depthFirstSearch(uint16_t start_node, std::array<bool, MAX_TOPOLOGY_NODES>& visited, uint16_t excluded_node) const;
    
    // Role determination helpers  
    AdaptiveNodeRole determineRoleFromConnectivity(uint16_t node_id, uint8_t neighbor_count) const;
    bool hasNeighborWithRole(uint16_t node_id, AdaptiveNodeRole role) const;
    
    // Debug helpers
    void logRoleChange(uint16_t node_id, AdaptiveNodeRole old_role, AdaptiveNodeRole new_role) const;
    void logTopologyUpdate(const TopologyNode& node, bool is_new) const;
};

#endif // TOPOLOGY_MANAGER_H_