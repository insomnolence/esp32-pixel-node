#include "topology_manager.h"
#include "esp_mac.h"
#include <algorithm>

const char* TopologyManager::TAG = "TopologyManager";

TopologyManager::TopologyManager()
    : local_node_id_(0)
    , neighbor_manager_(nullptr)
    , is_running_(false)
    , update_sequence_(0)
    , node_count_(0)
    , local_role_(AdaptiveNodeRole::UNKNOWN)
    , last_topology_update_ms_(0)
    , last_role_analysis_ms_(0)
    , topology_change_callback_(nullptr)
    , role_change_callback_(nullptr)
{
    topology_nodes_.fill(TopologyNode{});
}

TopologyManager::~TopologyManager() {
    stop();
}

esp_err_t TopologyManager::init(uint16_t local_node_id, NeighborManager* neighbor_manager) {
    if (!neighbor_manager) {
        ESP_LOGE(TAG, "NeighborManager pointer is null");
        return ESP_ERR_INVALID_ARG;
    }
    
    local_node_id_ = local_node_id;
    neighbor_manager_ = neighbor_manager;
    
    // Create entry for local node
    TopologyNode* local_node = findOrCreateNode(local_node_id_);
    if (local_node) {
        local_node->role = AdaptiveNodeRole::UNKNOWN;
        local_node->hop_count = 0;
        local_node->is_reachable = true;
        local_node->last_update_ms = getCurrentTimeMs();
    }
    
    ESP_LOGI(TAG, "Initialized TopologyManager for node %u", local_node_id_);
    return ESP_OK;
}

esp_err_t TopologyManager::start() {
    if (is_running_) {
        ESP_LOGW(TAG, "TopologyManager already running");
        return ESP_OK;
    }
    
    is_running_ = true;
    last_topology_update_ms_ = getCurrentTimeMs();
    last_role_analysis_ms_ = getCurrentTimeMs();
    
    ESP_LOGI(TAG, "Started topology management");
    return ESP_OK;
}

esp_err_t TopologyManager::stop() {
    if (!is_running_) {
        return ESP_OK;
    }
    
    is_running_ = false;
    ESP_LOGI(TAG, "Stopped topology management");
    return ESP_OK;
}

void TopologyManager::updateTopology() {
    if (!is_running_ || !neighbor_manager_) {
        return;
    }
    
    uint32_t current_time = getCurrentTimeMs();
    
    // Update topology from neighbor information
    std::array<NeighborInfo, MAX_NEIGHBORS> neighbors;
    size_t neighbor_count = neighbor_manager_->getAllNeighbors(neighbors);
    
    for (size_t i = 0; i < neighbor_count; ++i) {
        updateNodeFromNeighbor(neighbors[i]);
    }
    
    // Remove stale nodes
    removeStaleNodes();
    
    // Analyze network topology and roles
    if (current_time - last_role_analysis_ms_ >= 5000) { // Analyze every 5 seconds
        analyzeNetworkTopology();
        last_role_analysis_ms_ = current_time;
    }
    
    // Broadcast topology updates
    if (current_time - last_topology_update_ms_ >= TOPOLOGY_UPDATE_INTERVAL_MS) {
        broadcastTopologyUpdate();
        last_topology_update_ms_ = current_time;
    }
}

void TopologyManager::processTopologyUpdate(const uint8_t* sender_mac, const uint8_t* data, int len) {
    if (len < sizeof(TopologyUpdate)) {
        ESP_LOGW(TAG, "Received malformed topology update (len=%d, expected=%d)", 
                 len, sizeof(TopologyUpdate));
        return;
    }
    
    const TopologyUpdate* update = reinterpret_cast<const TopologyUpdate*>(data);
    
    // Validate update
    if (!isValidTopologyUpdate(*update)) {
        ESP_LOGW(TAG, "Received invalid topology update from " MACSTR, MAC2STR(sender_mac));
        return;
    }
    
    // Ignore our own updates
    if (update->sender_node_id == local_node_id_) {
        return;
    }
    
    stats_.topology_updates_received++;
    
    // Process nodes in the update
    for (uint8_t i = 0; i < update->node_count && i < 8; ++i) {
        const TopologyNode& remote_node = update->nodes[i];
        
        if (remote_node.node_id == 0 || remote_node.node_id == local_node_id_) {
            continue;
        }
        
        TopologyNode* local_node = findOrCreateNode(remote_node.node_id);
        if (!local_node) {
            ESP_LOGW(TAG, "Cannot add node %u - topology table full", remote_node.node_id);
            continue;
        }
        
        // Update node information if this is newer data
        bool is_new_node = !local_node->is_reachable;
        
        // Update with remote information (indirect knowledge)
        if (local_node->hop_count > remote_node.hop_count + 1) {
            local_node->hop_count = remote_node.hop_count + 1;
            local_node->is_reachable = true;
        }
        
        // Update role if we don't have better information
        if (local_node->role == AdaptiveNodeRole::UNKNOWN) {
            local_node->role = remote_node.role;
        }
        
        local_node->last_update_ms = getCurrentTimeMs();
        
        notifyTopologyChange(*local_node, is_new_node);
    }
    
    ESP_LOGV(TAG, "Processed topology update from node %u with %u nodes", 
             update->sender_node_id, update->node_count);
}

void TopologyManager::broadcastTopologyUpdate() {
    TopologyUpdate update{};
    update.magic = TOPOLOGY_UPDATE_MAGIC;
    update.sender_node_id = local_node_id_;
    update.update_sequence = update_sequence_++;
    update.topology_version = stats_.topology_version;
    
    // Pack active nodes into update (limited to 8 nodes per packet)
    uint8_t packed_count = 0;
    for (size_t i = 0; i < node_count_ && packed_count < 8; ++i) {
        const TopologyNode& node = topology_nodes_[i];
        if (node.is_reachable && node.node_id != local_node_id_) {
            update.nodes[packed_count] = node;
            packed_count++;
        }
    }
    update.node_count = packed_count;
    
    // Note: Actual transmission will be handled by the mesh coordinator
    // This method prepares the topology update structure
    
    stats_.topology_updates_sent++;
    stats_.last_full_update_ms = getCurrentTimeMs();
    
    ESP_LOGV(TAG, "Prepared topology update: seq=%u, nodes=%u", 
             update.update_sequence, update.node_count);
}

AdaptiveNodeRole TopologyManager::determineLocalRole() const {
    if (!neighbor_manager_) {
        return AdaptiveNodeRole::UNKNOWN;
    }
    
    size_t direct_neighbor_count = neighbor_manager_->getActiveNeighborCount();
    
    // Determine role based on connectivity
    return determineRoleFromConnectivity(local_node_id_, direct_neighbor_count);
}

void TopologyManager::updateNodeRole(uint16_t node_id, AdaptiveNodeRole role) {
    TopologyNode* node = findNode(node_id);
    if (!node) {
        return;
    }
    
    AdaptiveNodeRole old_role = node->role;
    if (old_role != role) {
        node->role = role;
        node->last_update_ms = getCurrentTimeMs();
        
        stats_.role_changes++;
        
        if (node_id == local_node_id_) {
            local_role_ = role;
        }
        
        notifyRoleChange(node_id, old_role, role);
        logRoleChange(node_id, old_role, role);
    }
}

AdaptiveNodeRole TopologyManager::getNodeRole(uint16_t node_id) const {
    const TopologyNode* node = getNode(node_id);
    return node ? node->role : AdaptiveNodeRole::UNKNOWN;
}

bool TopologyManager::isCriticalBridge() const {
    if (!neighbor_manager_) {
        return false;
    }
    
    // A node is a critical bridge if removing it would partition the network
    size_t connected_components_with_node = countConnectedComponents();
    size_t connected_components_without_node = countConnectedComponents(local_node_id_);
    
    return connected_components_without_node > connected_components_with_node;
}

bool TopologyManager::wouldPartitionNetwork(uint16_t node_id) const {
    return isNodeCriticalBridge(node_id);
}

size_t TopologyManager::getNetworkDiameter() const {
    size_t max_hops = 0;
    for (size_t i = 0; i < node_count_; ++i) {
        if (topology_nodes_[i].is_reachable && topology_nodes_[i].hop_count < 0xFF) {
            max_hops = std::max(max_hops, static_cast<size_t>(topology_nodes_[i].hop_count));
        }
    }
    return max_hops;
}

size_t TopologyManager::getReachableNodeCount() const {
    size_t count = 0;
    for (size_t i = 0; i < node_count_; ++i) {
        if (topology_nodes_[i].is_reachable) {
            count++;
        }
    }
    return count;
}

const TopologyNode* TopologyManager::getNode(uint16_t node_id) const {
    for (size_t i = 0; i < node_count_; ++i) {
        if (topology_nodes_[i].node_id == node_id) {
            return &topology_nodes_[i];
        }
    }
    return nullptr;
}

size_t TopologyManager::getAllNodes(std::array<TopologyNode, MAX_TOPOLOGY_NODES>& nodes) const {
    size_t active_count = 0;
    
    for (size_t i = 0; i < node_count_ && active_count < MAX_TOPOLOGY_NODES; ++i) {
        if (topology_nodes_[i].is_reachable) {
            nodes[active_count] = topology_nodes_[i];
            active_count++;
        }
    }
    
    // Sort by hop count (closest first)
    std::sort(nodes.begin(), nodes.begin() + active_count, 
              [](const TopologyNode& a, const TopologyNode& b) {
                  return a.hop_count < b.hop_count;
              });
    
    return active_count;
}

void TopologyManager::getNodesByRole(AdaptiveNodeRole role, std::array<uint16_t, MAX_TOPOLOGY_NODES>& node_ids, size_t& count) const {
    count = 0;
    
    for (size_t i = 0; i < node_count_ && count < MAX_TOPOLOGY_NODES; ++i) {
        if (topology_nodes_[i].is_reachable && topology_nodes_[i].role == role) {
            node_ids[count] = topology_nodes_[i].node_id;
            count++;
        }
    }
}

void TopologyManager::identifyBridgeNodes() {
    stats_.bridge_nodes = 0;
    
    for (size_t i = 0; i < node_count_; ++i) {
        TopologyNode& node = topology_nodes_[i];
        if (!node.is_reachable) continue;
        
        // Determine if node is a bridge based on its connectivity
        if (node.neighbor_count >= 2) {
            AdaptiveNodeRole calculated_role = calculateNodeRole(node);
            if (calculated_role == AdaptiveNodeRole::BRIDGE) {
                if (node.role != AdaptiveNodeRole::BRIDGE) {
                    updateNodeRole(node.node_id, AdaptiveNodeRole::BRIDGE);
                }
                stats_.bridge_nodes++;
            }
        }
    }
}

void TopologyManager::identifyCriticalBridges() {
    stats_.critical_bridges = 0;
    
    for (size_t i = 0; i < node_count_; ++i) {
        TopologyNode& node = topology_nodes_[i];
        if (!node.is_reachable) continue;
        
        if (isNodeCriticalBridge(node.node_id)) {
            if (node.role != AdaptiveNodeRole::CRITICAL_BRIDGE) {
                updateNodeRole(node.node_id, AdaptiveNodeRole::CRITICAL_BRIDGE);
            }
            stats_.critical_bridges++;
        }
    }
}

void TopologyManager::setTopologyChangeCallback(std::function<void(const TopologyNode&, bool)> callback) {
    topology_change_callback_ = callback;
}

void TopologyManager::setRoleChangeCallback(std::function<void(uint16_t, AdaptiveNodeRole, AdaptiveNodeRole)> callback) {
    role_change_callback_ = callback;
}

const TopologyManager::TopologyStats& TopologyManager::getStats() const {
    return stats_;
}

void TopologyManager::printTopologyMap() const {
    ESP_LOGI(TAG, "=== Network Topology (Nodes: %zu, Diameter: %zu) ===", 
             getReachableNodeCount(), getNetworkDiameter());
    
    for (size_t i = 0; i < node_count_; ++i) {
        const TopologyNode& node = topology_nodes_[i];
        if (!node.is_reachable) continue;
        
        uint32_t age_ms = getCurrentTimeMs() - node.last_update_ms;
        const char* role_str = getRoleString(node.role);
        
        ESP_LOGI(TAG, "  Node %u: %s hops=%u neighbors=%u RSSI=%d age=%lums", 
                 node.node_id, role_str, node.hop_count, node.neighbor_count,
                 node.best_rssi, age_ms);
    }
    
    ESP_LOGI(TAG, "Stats: updates_sent=%lu, received=%lu, bridges=%lu, critical=%lu",
             stats_.topology_updates_sent, stats_.topology_updates_received,
             stats_.bridge_nodes, stats_.critical_bridges);
}

const char* TopologyManager::getRoleString(AdaptiveNodeRole role) const {
    switch (role) {
        case AdaptiveNodeRole::UNKNOWN: return "UNKNOWN";
        case AdaptiveNodeRole::EDGE: return "EDGE";
        case AdaptiveNodeRole::BRIDGE: return "BRIDGE";
        case AdaptiveNodeRole::HUB: return "HUB";
        case AdaptiveNodeRole::CRITICAL_BRIDGE: return "CRITICAL_BRIDGE";
        default: return "INVALID";
    }
}

// Private methods

TopologyNode* TopologyManager::findOrCreateNode(uint16_t node_id) {
    // First, try to find existing node
    TopologyNode* existing = findNode(node_id);
    if (existing) {
        return existing;
    }
    
    // Create new entry if space available
    if (node_count_ < MAX_TOPOLOGY_NODES) {
        TopologyNode& new_node = topology_nodes_[node_count_++];
        new_node.node_id = node_id;
        new_node.last_update_ms = getCurrentTimeMs();
        return &new_node;
    }
    
    return nullptr; // Table full
}

TopologyNode* TopologyManager::findNode(uint16_t node_id) {
    for (size_t i = 0; i < node_count_; ++i) {
        if (topology_nodes_[i].node_id == node_id) {
            return &topology_nodes_[i];
        }
    }
    return nullptr;
}

void TopologyManager::updateNodeFromNeighbor(const NeighborInfo& neighbor) {
    TopologyNode* node = findOrCreateNode(neighbor.node_id);
    if (!node) {
        ESP_LOGW(TAG, "Cannot update node %u - topology table full", neighbor.node_id);
        return;
    }
    
    bool is_new_node = !node->is_reachable;
    
    // Update with direct neighbor information
    memcpy(node->mac_addr, neighbor.mac_addr, 6);
    node->is_reachable = neighbor.is_active;
    node->hop_count = 1; // Direct neighbor
    node->best_rssi = neighbor.average_rssi;
    node->last_update_ms = getCurrentTimeMs();
    
    // Update neighbor count from beacon info if available
    // Note: This would come from the neighbor's discovery beacons
    
    notifyTopologyChange(*node, is_new_node);
}

void TopologyManager::removeStaleNodes() {
    uint32_t current_time = getCurrentTimeMs();
    size_t removed_count = 0;
    
    for (size_t i = 0; i < node_count_; ++i) {
        TopologyNode& node = topology_nodes_[i];
        
        if (node.node_id == local_node_id_) {
            continue; // Never remove local node
        }
        
        if (node.is_reachable && 
            current_time - node.last_update_ms > TOPOLOGY_NODE_TIMEOUT_MS) {
            
            ESP_LOGI(TAG, "Node %u timed out from topology", node.node_id);
            node.is_reachable = false;
            removed_count++;
            
            if (topology_change_callback_) {
                topology_change_callback_(node, false);
            }
        }
    }
    
    if (removed_count > 0) {
        ESP_LOGD(TAG, "Removed %zu stale nodes from topology", removed_count);
    }
}

void TopologyManager::analyzeNetworkTopology() {
    // Update local role
    AdaptiveNodeRole new_local_role = determineLocalRole();
    if (new_local_role != local_role_) {
        updateNodeRole(local_node_id_, new_local_role);
    }
    
    // Identify bridge nodes
    identifyBridgeNodes();
    
    // Identify critical bridges  
    identifyCriticalBridges();
    
    // Update topology version
    stats_.topology_version++;
}

AdaptiveNodeRole TopologyManager::calculateNodeRole(const TopologyNode& node) const {
    return determineRoleFromConnectivity(node.node_id, node.neighbor_count);
}

void TopologyManager::notifyTopologyChange(const TopologyNode& node, bool is_new) {
    if (topology_change_callback_) {
        topology_change_callback_(node, is_new);
    }
    
    logTopologyUpdate(node, is_new);
}

void TopologyManager::notifyRoleChange(uint16_t node_id, AdaptiveNodeRole old_role, AdaptiveNodeRole new_role) {
    if (role_change_callback_) {
        role_change_callback_(node_id, old_role, new_role);
    }
}

uint32_t TopologyManager::getCurrentTimeMs() const {
    return esp_timer_get_time() / 1000;
}

bool TopologyManager::isValidTopologyUpdate(const TopologyUpdate& update) const {
    return update.magic == TOPOLOGY_UPDATE_MAGIC && 
           update.sender_node_id != 0 && 
           update.node_count <= 8;
}

bool TopologyManager::isNodeCriticalBridge(uint16_t node_id) const {
    if (node_id == local_node_id_) {
        return isCriticalBridge();
    }
    
    size_t components_with_node = countConnectedComponents();
    size_t components_without_node = countConnectedComponents(node_id);
    
    return components_without_node > components_with_node;
}

size_t TopologyManager::countConnectedComponents(uint16_t excluded_node) const {
    std::array<bool, MAX_TOPOLOGY_NODES> visited;
    visited.fill(false);
    
    size_t components = 0;
    
    for (size_t i = 0; i < node_count_; ++i) {
        const TopologyNode& node = topology_nodes_[i];
        if (!node.is_reachable || node.node_id == excluded_node) {
            continue;
        }
        
        size_t node_index = &node - topology_nodes_.data();
        if (!visited[node_index]) {
            depthFirstSearch(node.node_id, visited, excluded_node);
            components++;
        }
    }
    
    return components;
}

void TopologyManager::depthFirstSearch(uint16_t start_node, std::array<bool, MAX_TOPOLOGY_NODES>& visited, uint16_t excluded_node) const {
    // Find start node index
    size_t start_index = MAX_TOPOLOGY_NODES;
    for (size_t i = 0; i < node_count_; ++i) {
        if (topology_nodes_[i].node_id == start_node) {
            start_index = i;
            break;
        }
    }
    
    if (start_index == MAX_TOPOLOGY_NODES || visited[start_index]) {
        return;
    }
    
    visited[start_index] = true;
    
    // Visit all connected neighbors
    // Note: In a complete implementation, this would traverse actual connectivity
    // For now, we mark nodes as visited based on hop count proximity
    for (size_t i = 0; i < node_count_; ++i) {
        const TopologyNode& node = topology_nodes_[i];
        if (node.is_reachable && 
            node.node_id != excluded_node && 
            node.node_id != start_node &&
            !visited[i] &&
            node.hop_count <= topology_nodes_[start_index].hop_count + 1) {
            depthFirstSearch(node.node_id, visited, excluded_node);
        }
    }
}

AdaptiveNodeRole TopologyManager::determineRoleFromConnectivity(uint16_t node_id, uint8_t neighbor_count) const {
    // Role determination based on neighbor count and position
    if (neighbor_count == 0) {
        return AdaptiveNodeRole::UNKNOWN;
    } else if (neighbor_count == 1) {
        return AdaptiveNodeRole::EDGE;
    } else if (neighbor_count >= 5) {
        return AdaptiveNodeRole::HUB;
    } else {
        // Check if this node bridges different network segments
        if (isNodeCriticalBridge(node_id)) {
            return AdaptiveNodeRole::CRITICAL_BRIDGE;
        } else if (neighbor_count >= 2) {
            return AdaptiveNodeRole::BRIDGE;
        } else {
            return AdaptiveNodeRole::EDGE;
        }
    }
}

bool TopologyManager::hasNeighborWithRole(uint16_t node_id, AdaptiveNodeRole role) const {
    // This would check if a node has neighbors with a specific role
    // Implementation would require more detailed neighbor role tracking
    return false;
}

void TopologyManager::logRoleChange(uint16_t node_id, AdaptiveNodeRole old_role, AdaptiveNodeRole new_role) const {
    ESP_LOGI(TAG, "Node %u role changed: %s -> %s", 
             node_id, getRoleString(old_role), getRoleString(new_role));
}

void TopologyManager::logTopologyUpdate(const TopologyNode& node, bool is_new) const {
    ESP_LOGD(TAG, "%s node %u: %s hops=%u RSSI=%d", 
             is_new ? "New" : "Updated",
             node.node_id, getRoleString(node.role), 
             node.hop_count, node.best_rssi);
}