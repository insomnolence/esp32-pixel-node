#ifndef ESPNOW_MESH_COORDINATOR_H_
#define ESPNOW_MESH_COORDINATOR_H_

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "../packet/generic_packet.h"
#include <functional>
#include <memory>
#include <set>
#include <vector>
#include <ctime>

// Forward declarations for adaptive mesh components
class NeighborManager;
class TopologyManager; 
class AdaptiveRouter;

#define ESPNOW_MESH_MAX_PAYLOAD_LEN 200
#define ESPNOW_MESH_DEFAULT_TTL 4
#define ESPNOW_MESH_CHANNEL 6

enum class NodeRole {
    MESH_CLIENT,             // Normal mesh node
    MESH_ROOT_ACTIVE,        // Current mesh root with BLE
    MESH_ROOT_AUTONOMOUS     // Autonomous root without BLE
};

enum class MeshPacketType : uint8_t {
    LED_PATTERN = 0x01,      // LED pattern/color updates (high priority)
    NODE_STATUS = 0x02,      // Node health/status (low priority)
    NETWORK_BEACON = 0x03,   // Network discovery/maintenance
    ROOT_ELECTION = 0x04,    // Autonomous root election (legacy)
    ROOT_ANNOUNCEMENT = 0x05,// Root node announcement
    ELECTION_DISCOVERY = 0x06,   // New election system: Discovery phase
    ELECTION_CANDIDATE = 0x07,   // New election system: Candidate announcement  
    ELECTION_VOTE = 0x08,        // New election system: Vote casting
    ELECTION_RESULT = 0x09,      // New election system: Result announcement
    
    // Adaptive Mesh Packet Types (0x10-0x1F range)
    ADAPTIVE_NEIGHBOR_DISCOVERY = 0x10, // Neighbor discovery beacon with RSSI
    ADAPTIVE_TOPOLOGY_UPDATE = 0x11,    // Network topology information sharing
    ADAPTIVE_ROUTE_REQUEST = 0x12,      // Multi-path route discovery
    ADAPTIVE_ROUTE_REPLY = 0x13,        // Route discovery response
    ADAPTIVE_DATA_FORWARD = 0x14        // Smart packet forwarding with routing
};

enum class ElectionState : uint8_t {
    ELECTION_IDLE = 0,       // Not participating in election
    ELECTION_DISCOVERY,      // Listening for other candidates (3-7s)
    ELECTION_CANDIDATE,      // Announcing candidacy (2-4s)
    ELECTION_VOTING,         // Evaluating candidates and voting (2-3s)
    ELECTION_CONFIRMED       // Election complete, transitioning to final role
};

struct ESPNowMeshPacket {
    uint32_t packet_id;          // Unique packet identifier (prevents loops)
    uint8_t packet_type;         // MeshPacketType
    uint8_t ttl;                 // Time-to-live (hop count)
    uint8_t source_mac[6];       // Original sender MAC
    uint32_t timestamp;          // Send timestamp for sync
    uint16_t data_len;           // Actual payload size
    uint8_t data[ESPNOW_MESH_MAX_PAYLOAD_LEN - 20]; // Reserve space for header
} __attribute__((packed));

class BoundedPacketTracker {
public:
    BoundedPacketTracker();
    bool isPacketSeen(uint32_t packet_id);
    void cleanup();
    
private:
    static const size_t PACKET_HISTORY_SIZE = 128; // Power of 2 for fast modulo
    static const uint32_t CLEANUP_INTERVAL_MS = 30000; // 30 seconds
    
    uint32_t packet_history[PACKET_HISTORY_SIZE];
    size_t history_index;
    size_t history_count;
    uint32_t last_cleanup;
};

class ESPNowMeshCoordinator {
public:
    ESPNowMeshCoordinator();
    ~ESPNowMeshCoordinator();

    esp_err_t init();
    esp_err_t start();
    esp_err_t stop();
    
    // Role management
    NodeRole getCurrentRole() const;
    bool isRootNode() const;
    bool isBleConnected() const { return ble_connected; }
    
    // BLE integration - maintains same interface as old MeshCoordinator
    void onBleConnected();
    void onBleDisconnected();
    
    // Mesh communication - optimized for LED patterns
    esp_err_t sendGenericPacket(const GenericPacket& packet);
    esp_err_t sendLEDPattern(const GenericPacket& pattern);
    
    // Callbacks - same interface as old MeshCoordinator
    void setPacketCallback(std::function<void(const GenericPacket&)> callback);
    void setRoleChangeCallback(std::function<void(NodeRole, NodeRole)> callback);
    
    // Network info
    uint16_t getNodeId() const;
    const char* getRoleString() const;
    
    // Autonomous root election
    void startRootElection();
    bool isAutonomousRoot() const;
    void checkForRootElection();
    void sendRootAnnouncement();
    
    // Root stepDown functionality
    void stepDownIfNeeded();
    
    // BLE connection priority comparison
    uint32_t getBleConnectionAge() const;
    
    // BLE stabilization - prevent connections immediately after autonomous election
    bool shouldAcceptBleConnection() const;
    
    // Advanced election system
    void startAdvancedElection();
    void processElectionPacket(const ESPNowMeshPacket& packet);
    uint32_t calculateNodePriority() const;
    void checkElectionTimeout();
    
    // Adaptive mesh control (parallel implementation)
    esp_err_t enableAdaptiveMesh();
    void disableAdaptiveMesh();
    bool isAdaptiveMeshEnabled() const;
    
    // Memory and performance monitoring
    size_t getActiveNeighborCount() const;
    
    // Adaptive mesh status and monitoring
    void printAdaptiveMeshStatus() const;
    
    // Testing and simulation (for single-device validation)
    void simulateNeighborDiscovery(uint16_t simulated_node_id, int8_t rssi);
    
    // Network health monitoring
    struct NetworkStats {
        uint32_t packets_sent = 0;
        uint32_t packets_received = 0;
        uint32_t packets_dropped = 0;
        uint32_t send_failures = 0;
        uint32_t last_activity_ms = 0;
    };
    
    const NetworkStats& getNetworkStats() const;

    // Election candidate information
    struct ElectionCandidate {
        uint16_t node_id;
        uint32_t priority_score;
        uint32_t uptime_ms;
        uint8_t mac_addr[6];
        uint32_t timestamp_received;
        
        bool operator<(const ElectionCandidate& other) const {
            // Higher priority score wins, use node_id as tiebreaker
            if (priority_score != other.priority_score) {
                return priority_score > other.priority_score;
            }
            return node_id < other.node_id;  // Lower node_id wins ties
        }
    };

private:
    static const char* TAG;
    static ESPNowMeshCoordinator* instance;
    static SemaphoreHandle_t instance_mutex;
    
    NodeRole current_role;
    uint16_t node_id;
    bool ble_connected;
    uint32_t packet_counter;
    uint8_t local_mac[6];
    
    TaskHandle_t mesh_task_handle;
    BoundedPacketTracker packet_tracker;
    NetworkStats network_stats;
    
    // Autonomous root election
    uint32_t election_timer;
    uint32_t last_root_announcement;
    bool heard_from_root;
    
    // BLE connection timestamp tracking for priority comparison
    uint32_t ble_connection_uptime_ms;  // When BLE connected (our uptime)
    bool has_ble_connection_timestamp;  // Validity flag for timestamp
    
    // Advanced election system state
    ElectionState election_state;
    uint32_t election_start_time;
    uint32_t election_phase_timeout;
    static const size_t MAX_ELECTION_CANDIDATES = 16; // Bounded election candidates
    ElectionCandidate election_candidates[MAX_ELECTION_CANDIDATES];
    size_t election_candidate_count;
    uint16_t elected_root_node_id;
    uint32_t last_election_attempt;
    uint32_t autonomous_root_timestamp;  // Track when we became autonomous root for BLE stabilization
    
    std::function<void(const GenericPacket&)> packet_callback;
    std::function<void(NodeRole, NodeRole)> role_change_callback;
    
    // Adaptive mesh components (parallel implementation)
    std::unique_ptr<NeighborManager> neighbor_manager;
    std::unique_ptr<TopologyManager> topology_manager;
    std::unique_ptr<AdaptiveRouter> adaptive_router;
    bool adaptive_mesh_enabled;
    
    // ESP-NOW callbacks
    static void onESPNowSent(const uint8_t *mac_addr, esp_now_send_status_t status);
    static void onESPNowReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
    
    // Internal packet handling
    void handleReceivedPacket(const uint8_t *mac_addr, const uint8_t *data, int len);
    void forwardPacket(const ESPNowMeshPacket& packet);
    esp_err_t sendMeshPacket(const ESPNowMeshPacket& packet);
    esp_err_t sendMeshPacketWithRetry(const ESPNowMeshPacket& packet, int max_retries);
    esp_err_t sendMeshPacketHighPriority(const ESPNowMeshPacket& packet); // Fast path for time-sensitive data
    
    // Adaptive mesh unicast transmission
    esp_err_t sendMeshPacketToNode(const ESPNowMeshPacket& packet, uint16_t target_node_id);
    esp_err_t sendMeshPacketToAllNeighbors(const ESPNowMeshPacket& packet);
    
    // Packet creation
    ESPNowMeshPacket createMeshPacket(MeshPacketType type, const GenericPacket& payload, uint8_t ttl = ESPNOW_MESH_DEFAULT_TTL);
    uint32_t generatePacketId();
    
    // Network management
    esp_err_t initWiFi();
    esp_err_t initESPNow();
    esp_err_t transitionToRole(NodeRole new_role);
    
    // Random backoff to prevent collisions
    void randomBackoff();
    
    // Autonomous root election methods
    void becomeAutonomousRoot();
    esp_err_t sendElectionPacket();
    
    // Advanced election helper methods
    void sendElectionDiscoveryPacket();
    void sendElectionCandidatePacket();
    void sendElectionVotePacket();
    void sendElectionResultPacket();
    void processElectionDiscovery(const ESPNowMeshPacket& packet);
    void processElectionCandidate(const ESPNowMeshPacket& packet);
    void processElectionVote(const ESPNowMeshPacket& packet);
    void processElectionResult(const ESPNowMeshPacket& packet);
    void advanceElectionPhase();
    ElectionCandidate selectBestCandidate() const;
    
    // Adaptive mesh packet processing methods
    void processAdaptiveNeighborDiscovery(const ESPNowMeshPacket& packet);
    void processAdaptiveTopologyUpdate(const ESPNowMeshPacket& packet);
    void processAdaptiveRouting(const ESPNowMeshPacket& packet);
    void processAdaptiveDataForward(const ESPNowMeshPacket& packet);
};

#endif // ESPNOW_MESH_COORDINATOR_H_