#ifndef ROUTER_H_
#define ROUTER_H_

#include "neighbor_manager.h"
#include "topology_manager.h"
#include "esp_err.h"
#include "esp_log.h"
#include <array>
#include <functional>

// Maximum routes to maintain per destination
#define MAX_ROUTES_PER_DESTINATION 4
#define MAX_ROUTE_DESTINATIONS 32
#define MAX_ROUTE_HOP_COUNT 8

enum class RouteQuality : uint8_t {
    EXCELLENT = 0, // Direct neighbor with excellent link
    GOOD = 1,      // Direct neighbor with good link or 2-hop via excellent
    FAIR = 2,      // Multi-hop with fair overall quality
    POOR = 3,      // Multi-hop with poor overall quality
    UNUSABLE = 4   // Route not viable for data transmission
};

enum class PacketPriority : uint8_t {
    LOW = 0,       // Background traffic, can use any route
    NORMAL = 1,    // Standard mesh traffic
    HIGH = 2,      // LED patterns, time-sensitive
    CRITICAL = 3   // Network control, always use best route
};

struct RouteInfo {
    uint16_t destination_node;     // Target node ID
    uint16_t next_hop_node;        // Next node in route
    uint8_t hop_count;             // Total hops to destination
    RouteQuality quality;          // Overall route quality
    int8_t path_rssi;              // Worst RSSI along path
    uint32_t last_used_ms;         // When route was last used
    uint32_t last_updated_ms;      // When route info was updated
    bool is_active;                // Whether route is currently viable
    
    // Path information for loop detection
    uint16_t path_nodes[MAX_ROUTE_HOP_COUNT]; // Nodes in the path
    uint8_t path_length;                      // Number of nodes in path
    
    RouteInfo() {
        destination_node = 0;
        next_hop_node = 0;
        hop_count = 0xFF;
        quality = RouteQuality::UNUSABLE;
        path_rssi = -100;
        last_used_ms = 0;
        last_updated_ms = 0;
        is_active = false;
        memset(path_nodes, 0, sizeof(path_nodes));
        path_length = 0;
    }
    
    // Comparison for route quality (better routes first)
    bool operator<(const RouteInfo& other) const {
        if (quality != other.quality) {
            return quality < other.quality; // Better quality first
        }
        if (hop_count != other.hop_count) {
            return hop_count < other.hop_count; // Shorter routes first
        }
        return path_rssi > other.path_rssi; // Better RSSI first
    }
};

struct RouteDestination {
    uint16_t destination_node;
    std::array<RouteInfo, MAX_ROUTES_PER_DESTINATION> routes;
    uint8_t route_count;
    uint8_t primary_route_index;     // Index of best route
    uint32_t last_route_discovery;   // When routes were last updated
    
    RouteDestination() {
        destination_node = 0;
        route_count = 0;
        primary_route_index = 0;
        last_route_discovery = 0;
        routes.fill(RouteInfo{});
    }
};

struct RouteUpdatePacket {
    uint32_t magic;                  // Magic number for validation
    uint16_t sender_node_id;         // Who sent this update
    uint8_t update_sequence;         // Sequence number
    uint8_t route_count;             // Number of routes in this packet
    
    struct RouteEntry {
        uint16_t destination_node;
        uint8_t hop_count;
        int8_t path_quality;         // Worst RSSI in path
        uint16_t next_hop;           // Next hop from sender's perspective
    } routes[6];                     // Fit multiple routes per packet
} __attribute__((packed));

class AdaptiveRouter {
public:
    AdaptiveRouter();
    ~AdaptiveRouter();
    
    // Lifecycle management
    esp_err_t init(uint16_t local_node_id, NeighborManager* neighbor_manager, TopologyManager* topology_manager);
    esp_err_t start();
    esp_err_t stop();
    
    // Route discovery and management
    void updateRoutes();
    void discoverRoutes();
    void processRouteUpdate(const uint8_t* sender_mac, const uint8_t* data, int len);
    void broadcastRouteUpdate();
    
    // Route selection and forwarding
    uint16_t selectBestNextHop(uint16_t destination, PacketPriority priority);
    RouteInfo* findBestRoute(uint16_t destination, PacketPriority priority);
    bool hasRouteTo(uint16_t destination) const;
    
    // Route quality assessment
    RouteQuality calculateRouteQuality(uint8_t hop_count, int8_t worst_rssi) const;
    RouteQuality assessPathQuality(const uint16_t* path, uint8_t path_length) const;
    
    // Multi-path routing
    size_t getAllRoutesTo(uint16_t destination, std::array<RouteInfo*, MAX_ROUTES_PER_DESTINATION>& routes);
    uint16_t selectAlternateNextHop(uint16_t destination, uint16_t avoid_next_hop);
    void markRouteUnusable(uint16_t destination, uint16_t next_hop);
    
    // Route maintenance
    void validateRoutes();
    void removeStaleRoutes();
    void updateRouteUsage(uint16_t destination, uint16_t next_hop);
    
    // Load balancing
    uint16_t selectLoadBalancedNextHop(uint16_t destination);
    void recordRouteLoad(uint16_t next_hop, size_t packet_size);
    
    // Callbacks
    void setRouteChangeCallback(std::function<void(uint16_t, bool)> callback);
    
    // Statistics and debugging
    struct RoutingStats {
        uint32_t route_discoveries = 0;
        uint32_t route_updates_sent = 0;
        uint32_t route_updates_received = 0;
        uint32_t packets_routed = 0;
        uint32_t route_failures = 0;
        uint32_t alternate_routes_used = 0;
        uint16_t active_destinations = 0;
        uint16_t total_routes = 0;
        uint32_t last_full_discovery_ms = 0;
    };
    
    const RoutingStats& getStats() const;
    void printRoutingTable() const;
    const char* getRouteQualityString(RouteQuality quality) const;
    const char* getPacketPriorityString(PacketPriority priority) const;
    
private:
    static const char* TAG;
    static const uint32_t ROUTE_UPDATE_MAGIC = 0xCAFEBABE;
    static const uint32_t ROUTE_DISCOVERY_INTERVAL_MS = 15000;  // 15 seconds
    static const uint32_t ROUTE_TIMEOUT_MS = 25000;             // 25 seconds (< 30s neighbor timeout)
    static const uint32_t ROUTE_UPDATE_INTERVAL_MS = 20000;     // 20 seconds
    
    uint16_t local_node_id_;
    NeighborManager* neighbor_manager_;
    TopologyManager* topology_manager_;
    bool is_running_;
    uint8_t update_sequence_;
    
    // Routing table - bounded arrays for memory safety
    std::array<RouteDestination, MAX_ROUTE_DESTINATIONS> routing_table_;
    size_t destination_count_;
    
    // Load balancing helpers
    struct NodeLoad {
        uint16_t node_id;
        uint32_t bytes_forwarded;
        uint32_t packets_forwarded;
        uint32_t last_update_ms;
    };
    
    // Load balancing state
    std::array<NodeLoad, MAX_NEIGHBORS> node_loads_;
    size_t load_entry_count_;
    
    // Timing
    uint32_t last_route_discovery_ms_;
    uint32_t last_route_update_ms_;
    
    // Statistics
    RoutingStats stats_;
    
    // Callback
    std::function<void(uint16_t, bool)> route_change_callback_;
    
    // Internal route management
    RouteDestination* findOrCreateDestination(uint16_t destination_node);
    RouteDestination* findDestination(uint16_t destination_node);
    const RouteDestination* findDestination(uint16_t destination_node) const;
    RouteInfo* addRoute(RouteDestination* dest, const RouteInfo& route);
    void removeRoute(RouteDestination* dest, uint8_t route_index);
    void updatePrimaryRoute(RouteDestination* dest);
    
    // Route discovery algorithms
    void discoverDirectRoutes();
    void discoverMultiHopRoutes();
    void learnRoutesFromTopology();
    void propagateRouteUpdates();
    
    // Route validation
    bool isValidRoute(const RouteInfo& route) const;
    bool wouldCreateLoop(uint16_t destination, uint16_t next_hop) const;
    bool isRouteStale(const RouteInfo& route) const;
    
    // Load balancing helpers
    NodeLoad* findOrCreateNodeLoad(uint16_t node_id);
    uint32_t getNodeLoad(uint16_t node_id) const;
    void updateLoadBalancingMetrics();
    
    // Packet processing helpers
    void processRouteEntry(const RouteUpdatePacket::RouteEntry& entry, uint16_t sender_node);
    void mergeRouteUpdate(RouteDestination* dest, const RouteInfo& new_route);
    
    // Utility methods
    uint32_t getCurrentTimeMs() const;
    bool isValidRouteUpdate(const RouteUpdatePacket& update) const;
    void notifyRouteChange(uint16_t destination, bool is_available);
    
    // Debug helpers
    void logRouteDiscovery(const RouteInfo& route, bool is_new) const;
    void logRouteSelection(uint16_t destination, uint16_t next_hop, RouteQuality quality) const;
};

#endif // ROUTER_H_