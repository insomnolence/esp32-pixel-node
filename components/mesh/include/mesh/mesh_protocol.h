#pragma once

#include <stdint.h>
#include "sdkconfig.h"

#define ESPNOW_MESH_MAX_PAYLOAD_LEN 250
#define ESPNOW_MESH_DEFAULT_TTL 4
#define ESPNOW_MESH_CHANNEL 6

// =============================================================================
// MESH TIMING CONSTANTS
// Configurable via Kconfig with sensible defaults
// =============================================================================

// Time without hearing from root before considering it gone (default 15s)
#ifdef CONFIG_MESH_ROOT_TIMEOUT_MS
#define MESH_ROOT_TIMEOUT_MS CONFIG_MESH_ROOT_TIMEOUT_MS
#else
#define MESH_ROOT_TIMEOUT_MS 15000
#endif

// Time without hearing from BLE root before clearing network BLE status (default 30s)
#ifdef CONFIG_MESH_BLE_ROOT_TIMEOUT_MS
#define MESH_BLE_ROOT_TIMEOUT_MS CONFIG_MESH_BLE_ROOT_TIMEOUT_MS
#else
#define MESH_BLE_ROOT_TIMEOUT_MS 30000
#endif

// Cooldown after being displaced by another BLE root (default 5s)
#ifdef CONFIG_MESH_DISPLACEMENT_COOLDOWN_MS
#define MESH_DISPLACEMENT_COOLDOWN_MS CONFIG_MESH_DISPLACEMENT_COOLDOWN_MS
#else
#define MESH_DISPLACEMENT_COOLDOWN_MS 5000
#endif

enum class MeshPacketType : uint8_t {
    LED_PATTERN = 0x01,      // LED pattern/color updates
    ROOT_CLAIM = 0x02,       // Root election/handoff
    HEARTBEAT = 0x03         // Network statistics
};

enum class RootClaimReason : uint8_t {
    BLE_CONNECTED = 0x01,    // Highest priority - active BLE connection
    BLE_DISCONNECTED = 0x02, // Was BLE root, lost connection (grace period)
    BUTTON_PRESS = 0x03      // Manual takeover (only when no BLE in network)
    // FALLBACK removed - no automatic root election
};

// Common header for all packets
struct PacketHeader {
    uint8_t type;            // MeshPacketType
    uint32_t packet_id;      // Unique ID for dedup
    uint8_t ttl;             // Time to live
} __attribute__((packed));

struct LedPatternPacket {
    PacketHeader header;
    uint8_t pattern_type;
    uint8_t data_length;
    uint8_t pattern_data[240]; // Variable length
} __attribute__((packed));

struct RootClaimPacket {
    PacketHeader header;
    uint8_t node_id[6];      // MAC address of claiming node
    uint8_t reason;          // RootClaimReason
    uint8_t neighbor_count;  // For tiebreaking
    uint32_t timestamp;      // Claim time
} __attribute__((packed));

struct HeartbeatPacket {
    PacketHeader header;
    uint8_t node_id[6];         // MAC address of sender
    uint8_t neighbor_count;     // Direct neighbors
    int8_t avg_rssi;            // Average RSSI to neighbors
    uint32_t uptime_seconds;    // Node uptime
    uint8_t is_root;            // 1 if this node is root
    uint8_t has_ble_connection; // 1 if this node has active BLE connection
} __attribute__((packed));


