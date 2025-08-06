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
#include "../packet/generic_packet.h"
#include <functional>
#include <memory>
#include <set>
#include <ctime>

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
    ROOT_ELECTION = 0x04,    // Autonomous root election
    ROOT_ANNOUNCEMENT = 0x05 // Root node announcement
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

class PacketTracker {
public:
    bool isPacketSeen(uint32_t packet_id);
    void cleanup();
    
private:
    std::set<uint32_t> seen_packets;
    uint32_t last_cleanup = 0;
    static const uint32_t CLEANUP_INTERVAL_MS = 30000; // 30 seconds
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
    
    // Network health monitoring
    struct NetworkStats {
        uint32_t packets_sent = 0;
        uint32_t packets_received = 0;
        uint32_t packets_dropped = 0;
        uint32_t send_failures = 0;
        uint32_t last_activity_ms = 0;
    };
    
    const NetworkStats& getNetworkStats() const;

private:
    static const char* TAG;
    static ESPNowMeshCoordinator* instance;
    
    NodeRole current_role;
    uint16_t node_id;
    bool ble_connected;
    uint32_t packet_counter;
    uint8_t local_mac[6];
    
    TaskHandle_t mesh_task_handle;
    PacketTracker packet_tracker;
    NetworkStats network_stats;
    
    // Autonomous root election
    uint32_t election_timer;
    uint32_t last_root_announcement;
    bool heard_from_root;
    
    std::function<void(const GenericPacket&)> packet_callback;
    std::function<void(NodeRole, NodeRole)> role_change_callback;
    
    // ESP-NOW callbacks
    static void onESPNowSent(const uint8_t *mac_addr, esp_now_send_status_t status);
    static void onESPNowReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
    
    // Internal packet handling
    void handleReceivedPacket(const uint8_t *mac_addr, const uint8_t *data, int len);
    void forwardPacket(const ESPNowMeshPacket& packet);
    esp_err_t sendMeshPacket(const ESPNowMeshPacket& packet);
    esp_err_t sendMeshPacketWithRetry(const ESPNowMeshPacket& packet, int max_retries);
    esp_err_t sendMeshPacketHighPriority(const ESPNowMeshPacket& packet); // Fast path for time-sensitive data
    
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
};

#endif // ESPNOW_MESH_COORDINATOR_H_