#include "router.h"
#include "esp_mac.h"
#include <algorithm>

const char* AdaptiveRouter::TAG = "AdaptiveRouter";

AdaptiveRouter::AdaptiveRouter()
    : local_node_id_(0)
    , neighbor_manager_(nullptr)
    , topology_manager_(nullptr)
    , is_running_(false)
    , update_sequence_(0)
    , destination_count_(0)
    , load_entry_count_(0)
    , last_route_discovery_ms_(0)
    , last_route_update_ms_(0)
    , route_change_callback_(nullptr)
{
    routing_table_.fill(RouteDestination{});
    node_loads_.fill(NodeLoad{});
}

AdaptiveRouter::~AdaptiveRouter() {
    stop();
}

esp_err_t AdaptiveRouter::init(uint16_t local_node_id, NeighborManager* neighbor_manager, TopologyManager* topology_manager) {
    if (!neighbor_manager || !topology_manager) {
        ESP_LOGE(TAG, "Null pointer parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    local_node_id_ = local_node_id;
    neighbor_manager_ = neighbor_manager;
    topology_manager_ = topology_manager;
    
    ESP_LOGI(TAG, "Initialized AdaptiveRouter for node %u", local_node_id_);
    return ESP_OK;
}

esp_err_t AdaptiveRouter::start() {
    if (is_running_) {
        ESP_LOGW(TAG, "AdaptiveRouter already running");
        return ESP_OK;
    }
    
    is_running_ = true;
    last_route_discovery_ms_ = getCurrentTimeMs();
    last_route_update_ms_ = getCurrentTimeMs();
    
    ESP_LOGI(TAG, "Started adaptive routing");
    return ESP_OK;
}

esp_err_t AdaptiveRouter::stop() {
    if (!is_running_) {
        return ESP_OK;
    }
    
    is_running_ = false;
    ESP_LOGI(TAG, "Stopped adaptive routing");
    return ESP_OK;
}

void AdaptiveRouter::updateRoutes() {
    if (!is_running_) {
        return;
    }
    
    uint32_t current_time = getCurrentTimeMs();
    
    // Periodic route discovery
    if (current_time - last_route_discovery_ms_ >= ROUTE_DISCOVERY_INTERVAL_MS) {
        discoverRoutes();
        last_route_discovery_ms_ = current_time;
    }
    
    // Route validation and cleanup
    validateRoutes();
    removeStaleRoutes();
    
    // Broadcast route updates
    if (current_time - last_route_update_ms_ >= ROUTE_UPDATE_INTERVAL_MS) {
        broadcastRouteUpdate();
        last_route_update_ms_ = current_time;
    }
    
    // Update load balancing metrics
    updateLoadBalancingMetrics();
}

void AdaptiveRouter::discoverRoutes() {
    if (!neighbor_manager_ || !topology_manager_) {
        return;
    }
    
    stats_.route_discoveries++;
    stats_.last_full_discovery_ms = getCurrentTimeMs();
    
    // Discover direct routes to neighbors
    discoverDirectRoutes();
    
    // Learn routes from topology information
    learnRoutesFromTopology();
    
    // Discover multi-hop routes
    discoverMultiHopRoutes();
    
    ESP_LOGD(TAG, "Route discovery complete - %zu destinations", destination_count_);
}

void AdaptiveRouter::discoverDirectRoutes() {
    std::array<NeighborInfo, MAX_NEIGHBORS> neighbors;
    size_t neighbor_count = neighbor_manager_->getAllNeighbors(neighbors);
    
    for (size_t i = 0; i < neighbor_count; ++i) {
        const NeighborInfo& neighbor = neighbors[i];
        if (!neighbor.is_active || neighbor.link_quality == LinkQuality::UNUSABLE) {
            continue;
        }
        
        RouteInfo direct_route;
        direct_route.destination_node = neighbor.node_id;
        direct_route.next_hop_node = neighbor.node_id;
        direct_route.hop_count = 1;
        direct_route.path_rssi = neighbor.average_rssi;
        direct_route.quality = calculateRouteQuality(1, neighbor.average_rssi);
        direct_route.last_updated_ms = getCurrentTimeMs();
        direct_route.is_active = true;
        direct_route.path_nodes[0] = neighbor.node_id;
        direct_route.path_length = 1;
        
        RouteDestination* dest = findOrCreateDestination(neighbor.node_id);
        if (dest) {
            addRoute(dest, direct_route);
            logRouteDiscovery(direct_route, true);
        }
    }
}

void AdaptiveRouter::learnRoutesFromTopology() {
    std::array<TopologyNode, MAX_TOPOLOGY_NODES> nodes;
    size_t node_count = topology_manager_->getAllNodes(nodes);
    
    for (size_t i = 0; i < node_count; ++i) {
        const TopologyNode& node = nodes[i];
        if (!node.is_reachable || node.node_id == local_node_id_) {
            continue;
        }
        
        // Create route based on topology information
        RouteInfo topo_route;
        topo_route.destination_node = node.node_id;
        topo_route.hop_count = node.hop_count;
        topo_route.path_rssi = node.best_rssi;
        topo_route.quality = calculateRouteQuality(node.hop_count, node.best_rssi);
        topo_route.last_updated_ms = getCurrentTimeMs();
        topo_route.is_active = true;
        
        // For multi-hop routes, we need to find the next hop
        // This is simplified - in practice would use path information
        if (node.hop_count == 1) {
            topo_route.next_hop_node = node.node_id;
        } else {
            // Find best neighbor that can reach this destination
            std::array<NeighborInfo, MAX_NEIGHBORS> neighbors;
            size_t neighbor_count = neighbor_manager_->getAllNeighbors(neighbors);
            
            uint16_t best_next_hop = 0;
            RouteQuality best_quality = RouteQuality::UNUSABLE;
            
            for (size_t j = 0; j < neighbor_count; ++j) {
                const NeighborInfo& neighbor = neighbors[j];
                if (neighbor.is_active && neighbor.link_quality != LinkQuality::UNUSABLE) {
                    RouteQuality combined_quality = calculateRouteQuality(
                        node.hop_count, 
                        std::min(neighbor.average_rssi, node.best_rssi)
                    );
                    
                    if (combined_quality < best_quality) {
                        best_quality = combined_quality;
                        best_next_hop = neighbor.node_id;
                    }
                }
            }
            
            if (best_next_hop != 0) {
                topo_route.next_hop_node = best_next_hop;
                topo_route.quality = best_quality;
            } else {
                continue; // No viable next hop
            }
        }
        
        RouteDestination* dest = findOrCreateDestination(node.node_id);
        if (dest) {
            addRoute(dest, topo_route);
        }
    }
}

void AdaptiveRouter::discoverMultiHopRoutes() {
    // This would implement more sophisticated route discovery algorithms
    // For now, we rely on topology-based routing and route updates from neighbors
    ESP_LOGV(TAG, "Multi-hop route discovery - using topology and neighbor updates");
}

void AdaptiveRouter::processRouteUpdate(const uint8_t* sender_mac, const uint8_t* data, int len) {
    if (len < sizeof(RouteUpdatePacket)) {
        ESP_LOGW(TAG, "Received malformed route update (len=%d)", len);
        return;
    }
    
    const RouteUpdatePacket* update = reinterpret_cast<const RouteUpdatePacket*>(data);
    
    if (!isValidRouteUpdate(*update)) {
        ESP_LOGW(TAG, "Received invalid route update from " MACSTR, MAC2STR(sender_mac));
        return;
    }
    
    // Ignore our own updates
    if (update->sender_node_id == local_node_id_) {
        return;
    }
    
    stats_.route_updates_received++;
    
    // Process each route in the update
    for (uint8_t i = 0; i < update->route_count && i < 6; ++i) {
        processRouteEntry(update->routes[i], update->sender_node_id);
    }
    
    ESP_LOGV(TAG, "Processed route update from node %u with %u routes", 
             update->sender_node_id, update->route_count);
}

void AdaptiveRouter::processRouteEntry(const RouteUpdatePacket::RouteEntry& entry, uint16_t sender_node) {
    if (entry.destination_node == local_node_id_ || entry.destination_node == 0) {
        return; // Don't create routes to ourselves or invalid destinations
    }
    
    // Check if sender is a viable next hop
    const NeighborInfo* sender_neighbor = neighbor_manager_->getNeighborByNodeId(sender_node);
    if (!sender_neighbor || !sender_neighbor->is_active) {
        return; // Can't route through this node
    }
    
    // Create route via the sender
    RouteInfo learned_route;
    learned_route.destination_node = entry.destination_node;
    learned_route.next_hop_node = sender_node;
    learned_route.hop_count = entry.hop_count + 1; // Add one hop through sender
    learned_route.path_rssi = std::min(sender_neighbor->average_rssi, entry.path_quality);
    learned_route.quality = calculateRouteQuality(learned_route.hop_count, learned_route.path_rssi);
    learned_route.last_updated_ms = getCurrentTimeMs();
    learned_route.is_active = true;
    
    // Avoid creating loops
    if (wouldCreateLoop(entry.destination_node, sender_node)) {
        ESP_LOGV(TAG, "Rejecting route to %u via %u - would create loop", 
                 entry.destination_node, sender_node);
        return;
    }
    
    RouteDestination* dest = findOrCreateDestination(entry.destination_node);
    if (dest) {
        mergeRouteUpdate(dest, learned_route);
    }
}

void AdaptiveRouter::broadcastRouteUpdate() {
    RouteUpdatePacket update{};
    update.magic = ROUTE_UPDATE_MAGIC;
    update.sender_node_id = local_node_id_;
    update.update_sequence = update_sequence_++;
    
    // Pack best routes into update (limited to 6 routes per packet)
    uint8_t packed_count = 0;
    for (size_t i = 0; i < destination_count_ && packed_count < 6; ++i) {
        const RouteDestination& dest = routing_table_[i];
        if (dest.route_count > 0 && dest.destination_node != local_node_id_) {
            const RouteInfo& best_route = dest.routes[dest.primary_route_index];
            if (best_route.is_active && best_route.quality != RouteQuality::UNUSABLE) {
                update.routes[packed_count].destination_node = dest.destination_node;
                update.routes[packed_count].hop_count = best_route.hop_count;
                update.routes[packed_count].path_quality = best_route.path_rssi;
                update.routes[packed_count].next_hop = best_route.next_hop_node;
                packed_count++;
            }
        }
    }
    update.route_count = packed_count;
    
    // Note: Actual transmission handled by mesh coordinator
    stats_.route_updates_sent++;
    
    ESP_LOGV(TAG, "Prepared route update: seq=%u, routes=%u", 
             update.update_sequence, update.route_count);
}

uint16_t AdaptiveRouter::selectBestNextHop(uint16_t destination, PacketPriority priority) {
    RouteInfo* best_route = findBestRoute(destination, priority);
    if (!best_route || !best_route->is_active) {
        return 0; // No route available
    }
    
    updateRouteUsage(destination, best_route->next_hop_node);
    stats_.packets_routed++;
    
    logRouteSelection(destination, best_route->next_hop_node, best_route->quality);
    return best_route->next_hop_node;
}

RouteInfo* AdaptiveRouter::findBestRoute(uint16_t destination, PacketPriority priority) {
    RouteDestination* dest = findDestination(destination);
    if (!dest || dest->route_count == 0) {
        return nullptr;
    }
    
    // For critical packets, always use the best route
    if (priority == PacketPriority::CRITICAL) {
        RouteInfo& primary = dest->routes[dest->primary_route_index];
        return primary.is_active ? &primary : nullptr;
    }
    
    // For other priorities, consider load balancing
    if (priority == PacketPriority::HIGH) {
        // Use primary route for high priority
        RouteInfo& primary = dest->routes[dest->primary_route_index];
        return primary.is_active ? &primary : nullptr;
    }
    
    // For normal/low priority, use load balancing
    return &dest->routes[dest->primary_route_index];
}

bool AdaptiveRouter::hasRouteTo(uint16_t destination) const {
    const RouteDestination* dest = findDestination(destination);
    if (!dest) {
        return false;
    }
    
    for (uint8_t i = 0; i < dest->route_count; ++i) {
        if (dest->routes[i].is_active && dest->routes[i].quality != RouteQuality::UNUSABLE) {
            return true;
        }
    }
    
    return false;
}

RouteQuality AdaptiveRouter::calculateRouteQuality(uint8_t hop_count, int8_t worst_rssi) const {
    // Quality decreases with hop count and poor RSSI
    if (hop_count == 1) {
        // Direct neighbor - quality based on RSSI
        if (worst_rssi > -50) return RouteQuality::EXCELLENT;
        if (worst_rssi > -65) return RouteQuality::GOOD;
        if (worst_rssi > -75) return RouteQuality::FAIR;
        if (worst_rssi > -85) return RouteQuality::POOR;
        return RouteQuality::UNUSABLE;
    } else if (hop_count <= 3) {
        // Multi-hop routes - more conservative
        if (worst_rssi > -60) return RouteQuality::GOOD;
        if (worst_rssi > -70) return RouteQuality::FAIR;
        if (worst_rssi > -80) return RouteQuality::POOR;
        return RouteQuality::UNUSABLE;
    } else {
        // Long routes - even more conservative
        if (worst_rssi > -65) return RouteQuality::FAIR;
        if (worst_rssi > -75) return RouteQuality::POOR;
        return RouteQuality::UNUSABLE;
    }
}

size_t AdaptiveRouter::getAllRoutesTo(uint16_t destination, std::array<RouteInfo*, MAX_ROUTES_PER_DESTINATION>& routes) {
    RouteDestination* dest = findDestination(destination);
    if (!dest) {
        return 0;
    }
    
    size_t active_routes = 0;
    for (uint8_t i = 0; i < dest->route_count && active_routes < MAX_ROUTES_PER_DESTINATION; ++i) {
        if (dest->routes[i].is_active) {
            routes[active_routes] = &dest->routes[i];
            active_routes++;
        }
    }
    
    // Sort routes by quality (best first)
    std::sort(routes.begin(), routes.begin() + active_routes,
              [](const RouteInfo* a, const RouteInfo* b) {
                  return *a < *b;
              });
    
    return active_routes;
}

uint16_t AdaptiveRouter::selectAlternateNextHop(uint16_t destination, uint16_t avoid_next_hop) {
    std::array<RouteInfo*, MAX_ROUTES_PER_DESTINATION> routes;
    size_t route_count = getAllRoutesTo(destination, routes);
    
    for (size_t i = 0; i < route_count; ++i) {
        if (routes[i]->next_hop_node != avoid_next_hop && routes[i]->is_active) {
            stats_.alternate_routes_used++;
            updateRouteUsage(destination, routes[i]->next_hop_node);
            return routes[i]->next_hop_node;
        }
    }
    
    return 0; // No alternate route available
}

void AdaptiveRouter::markRouteUnusable(uint16_t destination, uint16_t next_hop) {
    RouteDestination* dest = findDestination(destination);
    if (!dest) {
        return;
    }
    
    for (uint8_t i = 0; i < dest->route_count; ++i) {
        if (dest->routes[i].next_hop_node == next_hop) {
            dest->routes[i].is_active = false;
            stats_.route_failures++;
            ESP_LOGW(TAG, "Marked route to %u via %u as unusable", destination, next_hop);
            
            // Update primary route if needed
            if (i == dest->primary_route_index) {
                updatePrimaryRoute(dest);
            }
            break;
        }
    }
}

void AdaptiveRouter::validateRoutes() {
    
    for (size_t i = 0; i < destination_count_; ++i) {
        RouteDestination& dest = routing_table_[i];
        bool routes_changed = false;
        
        for (uint8_t j = 0; j < dest.route_count; ++j) {
            RouteInfo& route = dest.routes[j];
            
            if (!route.is_active) {
                continue;
            }
            
            // Check if next hop neighbor is still available
            const NeighborInfo* next_hop_neighbor = neighbor_manager_->getNeighborByNodeId(route.next_hop_node);
            if (!next_hop_neighbor || !next_hop_neighbor->is_active) {
                route.is_active = false;
                routes_changed = true;
                ESP_LOGD(TAG, "Route to %u via %u invalidated - next hop unavailable", 
                         dest.destination_node, route.next_hop_node);
            }
            
            // Validate route based on other criteria
            if (!isValidRoute(route)) {
                route.is_active = false;
                routes_changed = true;
            }
        }
        
        if (routes_changed) {
            updatePrimaryRoute(&dest);
        }
    }
}

void AdaptiveRouter::removeStaleRoutes() {
    
    for (size_t i = 0; i < destination_count_; ++i) {
        RouteDestination& dest = routing_table_[i];
        
        for (uint8_t j = 0; j < dest.route_count; ++j) {
            if (isRouteStale(dest.routes[j])) {
                ESP_LOGD(TAG, "Removing stale route to %u via %u", 
                         dest.destination_node, dest.routes[j].next_hop_node);
                removeRoute(&dest, j);
                j--; // Adjust index after removal
            }
        }
    }
}

void AdaptiveRouter::updateRouteUsage(uint16_t destination, uint16_t next_hop) {
    RouteDestination* dest = findDestination(destination);
    if (!dest) {
        return;
    }
    
    for (uint8_t i = 0; i < dest->route_count; ++i) {
        if (dest->routes[i].next_hop_node == next_hop) {
            dest->routes[i].last_used_ms = getCurrentTimeMs();
            break;
        }
    }
    
    // Update load balancing metrics
    recordRouteLoad(next_hop, 0); // Size unknown at this level
}

uint16_t AdaptiveRouter::selectLoadBalancedNextHop(uint16_t destination) {
    std::array<RouteInfo*, MAX_ROUTES_PER_DESTINATION> routes;
    size_t route_count = getAllRoutesTo(destination, routes);
    
    if (route_count == 0) {
        return 0;
    }
    
    if (route_count == 1) {
        return routes[0]->next_hop_node;
    }
    
    // Select route with lowest load among good quality routes
    uint16_t best_next_hop = routes[0]->next_hop_node;
    uint32_t lowest_load = getNodeLoad(best_next_hop);
    
    for (size_t i = 1; i < route_count; ++i) {
        if (routes[i]->quality == routes[0]->quality) { // Same quality level
            uint32_t node_load = getNodeLoad(routes[i]->next_hop_node);
            if (node_load < lowest_load) {
                lowest_load = node_load;
                best_next_hop = routes[i]->next_hop_node;
            }
        } else {
            break; // Routes are sorted by quality, so we've found all with same quality
        }
    }
    
    return best_next_hop;
}

void AdaptiveRouter::recordRouteLoad(uint16_t next_hop, size_t packet_size) {
    NodeLoad* load_entry = findOrCreateNodeLoad(next_hop);
    if (load_entry) {
        load_entry->bytes_forwarded += packet_size;
        load_entry->packets_forwarded++;
        load_entry->last_update_ms = getCurrentTimeMs();
    }
}

void AdaptiveRouter::setRouteChangeCallback(std::function<void(uint16_t, bool)> callback) {
    route_change_callback_ = callback;
}

const AdaptiveRouter::RoutingStats& AdaptiveRouter::getStats() const {
    return stats_;
}

void AdaptiveRouter::printRoutingTable() const {
    ESP_LOGI(TAG, "=== Routing Table (Destinations: %zu) ===", destination_count_);
    
    for (size_t i = 0; i < destination_count_; ++i) {
        const RouteDestination& dest = routing_table_[i];
        if (dest.route_count == 0) continue;
        
        ESP_LOGI(TAG, "Destination %u (%u routes):", dest.destination_node, dest.route_count);
        
        for (uint8_t j = 0; j < dest.route_count; ++j) {
            const RouteInfo& route = dest.routes[j];
            const char* quality_str = getRouteQualityString(route.quality);
            const char* primary_marker = (j == dest.primary_route_index) ? "*" : " ";
            
            ESP_LOGI(TAG, "  %s via %u: hops=%u, RSSI=%d, %s %s", 
                     primary_marker,
                     route.next_hop_node, route.hop_count, route.path_rssi, 
                     quality_str, route.is_active ? "ACTIVE" : "INACTIVE");
        }
    }
    
    ESP_LOGI(TAG, "Stats: discoveries=%lu, routed=%lu, failures=%lu, alternates=%lu",
             stats_.route_discoveries, stats_.packets_routed, 
             stats_.route_failures, stats_.alternate_routes_used);
}

const char* AdaptiveRouter::getRouteQualityString(RouteQuality quality) const {
    switch (quality) {
        case RouteQuality::EXCELLENT: return "EXCELLENT";
        case RouteQuality::GOOD: return "GOOD";
        case RouteQuality::FAIR: return "FAIR";
        case RouteQuality::POOR: return "POOR";
        case RouteQuality::UNUSABLE: return "UNUSABLE";
        default: return "UNKNOWN";
    }
}

const char* AdaptiveRouter::getPacketPriorityString(PacketPriority priority) const {
    switch (priority) {
        case PacketPriority::LOW: return "LOW";
        case PacketPriority::NORMAL: return "NORMAL";
        case PacketPriority::HIGH: return "HIGH";
        case PacketPriority::CRITICAL: return "CRITICAL";
        default: return "UNKNOWN";
    }
}

// Private method implementations

RouteDestination* AdaptiveRouter::findOrCreateDestination(uint16_t destination_node) {
    // First try to find existing destination
    RouteDestination* existing = findDestination(destination_node);
    if (existing) {
        return existing;
    }
    
    // Create new destination if space available
    if (destination_count_ < MAX_ROUTE_DESTINATIONS) {
        RouteDestination& new_dest = routing_table_[destination_count_++];
        new_dest.destination_node = destination_node;
        new_dest.last_route_discovery = getCurrentTimeMs();
        stats_.active_destinations = destination_count_;
        return &new_dest;
    }
    
    return nullptr; // Table full
}

RouteDestination* AdaptiveRouter::findDestination(uint16_t destination_node) {
    for (size_t i = 0; i < destination_count_; ++i) {
        if (routing_table_[i].destination_node == destination_node) {
            return &routing_table_[i];
        }
    }
    return nullptr;
}

const RouteDestination* AdaptiveRouter::findDestination(uint16_t destination_node) const {
    for (size_t i = 0; i < destination_count_; ++i) {
        if (routing_table_[i].destination_node == destination_node) {
            return &routing_table_[i];
        }
    }
    return nullptr;
}

RouteInfo* AdaptiveRouter::addRoute(RouteDestination* dest, const RouteInfo& route) {
    if (!dest || dest->route_count >= MAX_ROUTES_PER_DESTINATION) {
        return nullptr;
    }
    
    // Check if we already have a route via this next hop
    for (uint8_t i = 0; i < dest->route_count; ++i) {
        if (dest->routes[i].next_hop_node == route.next_hop_node) {
            // Update existing route
            if (route.quality < dest->routes[i].quality || 
                route.hop_count < dest->routes[i].hop_count) {
                dest->routes[i] = route;
                updatePrimaryRoute(dest);
                // Update total route count
    uint16_t total = 0;
    for (size_t i = 0; i < destination_count_; ++i) {
        total += routing_table_[i].route_count;
    }
    stats_.total_routes = total;
                return &dest->routes[i];
            }
            return &dest->routes[i]; // Keep existing better route
        }
    }
    
    // Add new route
    dest->routes[dest->route_count] = route;
    dest->route_count++;
    updatePrimaryRoute(dest);
    // Update total route count
    uint16_t total = 0;
    for (size_t i = 0; i < destination_count_; ++i) {
        total += routing_table_[i].route_count;
    }
    stats_.total_routes = total;
    
    notifyRouteChange(dest->destination_node, true);
    
    return &dest->routes[dest->route_count - 1];
}

void AdaptiveRouter::removeRoute(RouteDestination* dest, uint8_t route_index) {
    if (!dest || route_index >= dest->route_count) {
        return;
    }
    
    // Shift routes down
    for (uint8_t i = route_index; i < dest->route_count - 1; ++i) {
        dest->routes[i] = dest->routes[i + 1];
    }
    
    dest->route_count--;
    
    // Clear the last slot
    dest->routes[dest->route_count] = RouteInfo{};
    
    // Update primary route index
    if (dest->primary_route_index >= dest->route_count) {
        dest->primary_route_index = dest->route_count > 0 ? 0 : 0;
    }
    updatePrimaryRoute(dest);
    
    // Update total route count
    uint16_t total = 0;
    for (size_t i = 0; i < destination_count_; ++i) {
        total += routing_table_[i].route_count;
    }
    stats_.total_routes = total;
    
    if (dest->route_count == 0) {
        notifyRouteChange(dest->destination_node, false);
    }
}

void AdaptiveRouter::updatePrimaryRoute(RouteDestination* dest) {
    if (!dest || dest->route_count == 0) {
        return;
    }
    
    uint8_t best_index = 0;
    for (uint8_t i = 1; i < dest->route_count; ++i) {
        if (dest->routes[i].is_active && dest->routes[i] < dest->routes[best_index]) {
            best_index = i;
        }
    }
    
    dest->primary_route_index = best_index;
}

void AdaptiveRouter::mergeRouteUpdate(RouteDestination* dest, const RouteInfo& new_route) {
    // This is a simplified merge - add route if it's better than existing
    addRoute(dest, new_route);
}

bool AdaptiveRouter::isValidRoute(const RouteInfo& route) const {
    return route.destination_node != 0 && 
           route.destination_node != local_node_id_ &&
           route.next_hop_node != 0 &&
           route.hop_count > 0 && 
           route.hop_count < 255 &&
           route.quality != RouteQuality::UNUSABLE;
}

bool AdaptiveRouter::wouldCreateLoop(uint16_t destination, uint16_t next_hop) const {
    // Simplified loop detection - check if next_hop is the destination
    return next_hop == destination;
}

bool AdaptiveRouter::isRouteStale(const RouteInfo& route) const {
    if (!route.is_active) {
        return true;
    }
    
    uint32_t current_time = getCurrentTimeMs();
    return (current_time - route.last_updated_ms) > ROUTE_TIMEOUT_MS;
}

AdaptiveRouter::NodeLoad* AdaptiveRouter::findOrCreateNodeLoad(uint16_t node_id) {
    // Find existing entry
    for (size_t i = 0; i < load_entry_count_; ++i) {
        if (node_loads_[i].node_id == node_id) {
            return &node_loads_[i];
        }
    }
    
    // Create new entry if space available
    if (load_entry_count_ < MAX_NEIGHBORS) {
        NodeLoad& new_entry = node_loads_[load_entry_count_++];
        new_entry.node_id = node_id;
        new_entry.bytes_forwarded = 0;
        new_entry.packets_forwarded = 0;
        new_entry.last_update_ms = getCurrentTimeMs();
        return &new_entry;
    }
    
    return nullptr;
}

uint32_t AdaptiveRouter::getNodeLoad(uint16_t node_id) const {
    for (size_t i = 0; i < load_entry_count_; ++i) {
        if (node_loads_[i].node_id == node_id) {
            return node_loads_[i].packets_forwarded;
        }
    }
    return 0;
}

void AdaptiveRouter::updateLoadBalancingMetrics() {
    uint32_t current_time = getCurrentTimeMs();
    
    // Age out old load data (simple decay)
    for (size_t i = 0; i < load_entry_count_; ++i) {
        NodeLoad& load = node_loads_[i];
        if (current_time - load.last_update_ms > 60000) { // 60 seconds
            load.bytes_forwarded = load.bytes_forwarded / 2;
            load.packets_forwarded = load.packets_forwarded / 2;
            load.last_update_ms = current_time;
        }
    }
}

uint32_t AdaptiveRouter::getCurrentTimeMs() const {
    return esp_timer_get_time() / 1000;
}

bool AdaptiveRouter::isValidRouteUpdate(const RouteUpdatePacket& update) const {
    return update.magic == ROUTE_UPDATE_MAGIC && 
           update.sender_node_id != 0 && 
           update.route_count <= 6;
}

void AdaptiveRouter::notifyRouteChange(uint16_t destination, bool is_available) {
    if (route_change_callback_) {
        route_change_callback_(destination, is_available);
    }
}


void AdaptiveRouter::logRouteDiscovery(const RouteInfo& route, bool is_new) const {
    ESP_LOGD(TAG, "%s route to %u via %u: hops=%u, RSSI=%d, %s", 
             is_new ? "Discovered" : "Updated",
             route.destination_node, route.next_hop_node, 
             route.hop_count, route.path_rssi,
             getRouteQualityString(route.quality));
}

void AdaptiveRouter::logRouteSelection(uint16_t destination, uint16_t next_hop, RouteQuality quality) const {
    ESP_LOGV(TAG, "Selected route to %u via %u (%s)", 
             destination, next_hop, getRouteQualityString(quality));
}