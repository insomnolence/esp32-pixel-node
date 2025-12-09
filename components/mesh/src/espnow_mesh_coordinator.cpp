#include "mesh/espnow_mesh_coordinator.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_random.h"
#include "nvs_flash.h"
#ifdef CONFIG_BT_ENABLED
#include "esp_bt.h"
#endif
#include "esp_mac.h"
#include "esp_rom_crc.h"
#include <string.h>
#include <ctime>

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Keep NeighborTracker for now
#include "mesh/neighbor_tracker.h"

const char* ESPNowMeshCoordinator::TAG = "ESPNowMeshCoordinator";
ESPNowMeshCoordinator* ESPNowMeshCoordinator::instance = nullptr;
SemaphoreHandle_t ESPNowMeshCoordinator::instance_mutex = nullptr;

// BoundedPacketTracker implementation
BoundedPacketTracker::BoundedPacketTracker()
    : history_index(0)
    , history_count(0)
    , last_cleanup(0)
    , tracker_mutex(nullptr)
{
    memset(packet_history, 0, sizeof(packet_history));
    tracker_mutex = xSemaphoreCreateMutex();
}

BoundedPacketTracker::~BoundedPacketTracker() {
    if (tracker_mutex) vSemaphoreDelete(tracker_mutex);
}

bool BoundedPacketTracker::isNewPacket(uint32_t packet_id) {
    if (!tracker_mutex || xSemaphoreTake(tracker_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return false;  // Can't acquire lock, treat as duplicate to be safe
    }

    // Check if packet already exists in history
    for (size_t i = 0; i < history_count; ++i) {
        if (packet_history[i] == packet_id) {
            xSemaphoreGive(tracker_mutex);
            return false;  // Duplicate packet
        }
    }

    // New packet - mark it as seen
    packet_history[history_index] = packet_id;
    history_index = (history_index + 1) % PACKET_HISTORY_SIZE;
    if (history_count < PACKET_HISTORY_SIZE) {
        history_count++;
    }

    xSemaphoreGive(tracker_mutex);
    return true;  // New packet
}

void BoundedPacketTracker::markPacketSeen(uint32_t packet_id) {
    if (!tracker_mutex || xSemaphoreTake(tracker_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }
    
    packet_history[history_index] = packet_id;
    history_index = (history_index + 1) % PACKET_HISTORY_SIZE;
    if (history_count < PACKET_HISTORY_SIZE) {
        history_count++;
    }
    
    xSemaphoreGive(tracker_mutex);
}

void BoundedPacketTracker::cleanup() {
    // No-op for circular buffer
}

// ESPNowMeshCoordinator implementation
ESPNowMeshCoordinator::ESPNowMeshCoordinator(MeshHardwareInterface* hw)
    : current_role(NodeRole::MESH_CLIENT)
    , node_id(0)
    , ble_connected(false)
    , current_claim_reason(RootClaimReason::BUTTON_PRESS) // Default to lowest priority (no FALLBACK anymore)
    , packet_counter(0)
    , mesh_task_handle(nullptr)
    , election_timer(0)
    , last_root_announcement(0)
    , heard_from_root(false)
    , ble_connection_uptime_ms(0)
    , has_ble_connection_timestamp(false)
    , network_has_ble_root(false)
    , last_ble_root_seen(0)
    , displaced_until(0)
    , last_heartbeat_sent(0)
    , heartbeat_interval_ms(10000) // Default start
    , hardware(hw)
{
    if (instance_mutex == nullptr) {
        instance_mutex = xSemaphoreCreateMutex();
    }
    instance = this;
    memset(local_mac, 0, 6);
    memset(current_root_mac, 0, 6);
    memset(ble_root_mac, 0, 6);

    // Initialize random seed (if not already done by system) mostly handled by hardware init
    // Random stagger for heartbeat
    if (hardware) {
        heartbeat_interval_ms += (hardware->getRandom() % 2000);
    }
}

ESPNowMeshCoordinator::~ESPNowMeshCoordinator() {
    stop();
    instance = nullptr;
    if (instance_mutex) {
        vSemaphoreDelete(instance_mutex);
        instance_mutex = nullptr;
    }
}

esp_err_t ESPNowMeshCoordinator::init() {
    ESP_LOGI(TAG, "Initializing ESP-NOW Mesh Coordinator (Simplified)");
    
    if (!hardware) {
        ESP_LOGE(TAG, "No hardware interface provided!");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = initWiFi();
    if (ret != ESP_OK) return ret;
    
    ret = initESPNow();
    if (ret != ESP_OK) return ret;
    
    hardware->getMacAddress(local_mac);
    node_id = (local_mac[4] << 8) | local_mac[5];
    
    ESP_LOGI(TAG, "Initialized Node ID: 0x%04X", node_id);
    
    // Initialize simplified NeighborTracker
    neighbor_tracker = std::make_unique<NeighborTracker>(node_id);
    
    return ESP_OK;
}

esp_err_t ESPNowMeshCoordinator::initWiFi() {
    return hardware->initWiFi();
}

esp_err_t ESPNowMeshCoordinator::initESPNow() {
    esp_err_t ret = hardware->initESPNow();
    if (ret != ESP_OK) return ret;
    
    // Register callback via hardware abstraction
    hardware->setReceiveCallback(onESPNowReceivedWrapper);
    
    // We don't register send cb in the abstraction for now as it wasn't used critically
    
    // Add broadcast peer
    return hardware->addPeer(BROADCAST_MAC, ESPNOW_MESH_CHANNEL, false);
}

// Static wrapper for the hardware callback
void ESPNowMeshCoordinator::onESPNowReceivedWrapper(const uint8_t *mac_addr, const uint8_t *data, int len, int8_t rssi) {
    if (instance_mutex && xSemaphoreTake(instance_mutex, 0)) {
        if (instance) {
            instance->handleReceivedPacket(mac_addr, data, len, rssi);
        }
        xSemaphoreGive(instance_mutex);
    }
}


esp_err_t ESPNowMeshCoordinator::start() {
    ESP_LOGI(TAG, "Starting Mesh Coordinator");
    return ESP_OK;
}

esp_err_t ESPNowMeshCoordinator::stop() {
    if (hardware) {
        return hardware->stopESPNow();
    }
    return ESP_OK;
}

// Role Management
NodeRole ESPNowMeshCoordinator::getCurrentRole() const { return current_role; }
bool ESPNowMeshCoordinator::isRootNode() const { return current_role != NodeRole::MESH_CLIENT; }
uint16_t ESPNowMeshCoordinator::getNodeId() const { return node_id; }

const char* ESPNowMeshCoordinator::getRoleString() const {
    switch (current_role) {
        case NodeRole::MESH_ROOT_ACTIVE: return "ROOT (BLE)";
        case NodeRole::MESH_ROOT_AUTONOMOUS: return "ROOT (Auto)";
        default: return "CLIENT";
    }
}

// BLE Integration
void ESPNowMeshCoordinator::onBleConnected() {
    ble_connected = true;
    ble_connection_uptime_ms = hardware->getMillis();
    has_ble_connection_timestamp = true;
    
    transitionToRole(NodeRole::MESH_ROOT_ACTIVE);
    current_claim_reason = RootClaimReason::BLE_CONNECTED;
    broadcastRootClaim(RootClaimReason::BLE_CONNECTED);
}

void ESPNowMeshCoordinator::onBleDisconnected() {
    ble_connected = false;
    has_ble_connection_timestamp = false;

    if (current_role == NodeRole::MESH_ROOT_ACTIVE) {
        ESP_LOGI(TAG, "BLE disconnected, transitioning to autonomous root (BLE_DISCONNECTED grace period).");
        transitionToRole(NodeRole::MESH_ROOT_AUTONOMOUS);
        current_claim_reason = RootClaimReason::BLE_DISCONNECTED;
        broadcastRootClaim(RootClaimReason::BLE_DISCONNECTED);
    }
}

esp_err_t ESPNowMeshCoordinator::transitionToRole(NodeRole new_role) {
    if (current_role == new_role) return ESP_OK;
    
    NodeRole old_role = current_role;
    current_role = new_role;
    
    if (new_role == NodeRole::MESH_ROOT_AUTONOMOUS) {
        autonomous_root_timestamp = hardware->getMillis();
    }
    
    if (role_change_callback) {
        role_change_callback(old_role, new_role);
    }
    return ESP_OK;
}

// Packet Handling
esp_err_t ESPNowMeshCoordinator::sendGenericPacket(const GenericPacket& packet) {
    return sendLEDPattern(packet);
}

esp_err_t ESPNowMeshCoordinator::sendLEDPattern(const GenericPacket& pattern) {
    if (!pattern.isValid()) return ESP_ERR_INVALID_ARG;

    LedPatternPacket packet;
    packet.header.type = (uint8_t)MeshPacketType::LED_PATTERN;
    packet.header.packet_id = generatePacketId();
    packet.header.ttl = ESPNOW_MESH_DEFAULT_TTL;

    packet.pattern_type = 1; // Default
    packet.data_length = std::min((size_t)240, pattern.getLength());
    memcpy(packet.pattern_data, pattern.getData(), packet.data_length);

    // Mark as seen so we don't process our own broadcast if we receive it back?
    // Actually receiving back from ourselves is filtered by MAC check.
    // But marking as seen prevents re-broadcasting if we hear it from neighbor.
    packet_tracker.markPacketSeen(packet.header.packet_id);

    size_t size = offsetof(LedPatternPacket, pattern_data) + packet.data_length;

    // Send LED pattern multiple times for reliability (ESP-NOW broadcasts can be lossy)
    // Use random jitter between retransmissions to reduce collision probability
    // when multiple nodes broadcast simultaneously
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < 3; i++) {
        ret = broadcastPacket(&packet, size);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "LED pattern broadcast attempt %d failed: %s", i + 1, esp_err_to_name(ret));
        }
        if (i < 2) {
            // Random jitter: 3-7ms between retransmissions
            vTaskDelay(pdMS_TO_TICKS(3 + (esp_random() % 5)));
        }
    }
    return ret;
}

esp_err_t ESPNowMeshCoordinator::broadcastPacket(const void* data, size_t len) {
    if (!hardware) return ESP_FAIL;
    
    esp_err_t ret = hardware->sendEspNow(BROADCAST_MAC, (uint8_t*)data, len);
    
    if (ret == ESP_OK) {
        network_stats.packets_sent++;
    } else {
        network_stats.send_failures++;
    }
    return ret;
}


void ESPNowMeshCoordinator::handleReceivedPacket(const uint8_t *mac_addr, const uint8_t *data, int len, int8_t rssi) {
    if (len < sizeof(PacketHeader)) return;

    // Update neighbor information
    if (neighbor_tracker) {
        neighbor_tracker->onPacketReceived(mac_addr, rssi);
    }
    
    const PacketHeader* header = (const PacketHeader*)data;
    
    // Dedup - isNewPacket checks and marks in one atomic operation
    if (!packet_tracker.isNewPacket(header->packet_id)) return;
    
    network_stats.packets_received++;
    network_stats.last_activity_ms = hardware->getMillis();
    
    // Forward immediately (flooding)
    forwardPacket(data, len);
    
    // Process locally
    if (header->type == (uint8_t)MeshPacketType::LED_PATTERN) {
        if (len < offsetof(LedPatternPacket, pattern_data)) return;
        const LedPatternPacket* p = (const LedPatternPacket*)data;
        
        if (len < offsetof(LedPatternPacket, pattern_data) + p->data_length) return;
        
        if (packet_callback) {
            GenericPacket gp(p->pattern_data, p->data_length);
            packet_callback(gp);
        }
    } else if (header->type == (uint8_t)MeshPacketType::ROOT_CLAIM) {
        if (len < sizeof(RootClaimPacket)) return;
        const RootClaimPacket* claim = (const RootClaimPacket*)data;

        ESP_LOGD(TAG, "Received ROOT_CLAIM from %02X:%02X:%02X:%02X:%02X:%02X, reason: %d, my_role: %s, my_reason: %d",
                 claim->node_id[0], claim->node_id[1], claim->node_id[2],
                 claim->node_id[3], claim->node_id[4], claim->node_id[5],
                 claim->reason, getRoleString(), (int)current_claim_reason);

        // Track BLE root status from ROOT_CLAIM packets
        if (claim->reason == (uint8_t)RootClaimReason::BLE_CONNECTED) {
            network_has_ble_root = true;
            memcpy(ble_root_mac, claim->node_id, 6);
            last_ble_root_seen = hardware->getMillis();
            ESP_LOGD(TAG, "BLE root detected from ROOT_CLAIM: %02X:%02X:%02X:%02X:%02X:%02X",
                     claim->node_id[0], claim->node_id[1], claim->node_id[2],
                     claim->node_id[3], claim->node_id[4], claim->node_id[5]);
        }

        // Check if we should yield to this claim
        if (isRootNode() && shouldYieldTo(*claim)) {
            ESP_LOGI(TAG, "Yielding root to node %02X:%02X:%02X:%02X:%02X:%02X, reason: %d",
                     claim->node_id[0], claim->node_id[1], claim->node_id[2],
                     claim->node_id[3], claim->node_id[4], claim->node_id[5],
                     claim->reason);

            // Handle BLE displacement (we have BLE, they claim BLE)
            if (ble_connected && claim->reason == (uint8_t)RootClaimReason::BLE_CONNECTED) {
                handleBleDisplacement(*claim);
            }

            transitionToRole(NodeRole::MESH_CLIENT);
            heard_from_root = true; // Acknowledge a new root is present
            last_root_announcement = hardware->getMillis(); // Reset timeout so we don't reclaim root immediately
            updateCurrentRoot(claim->node_id);
        } else if (!isRootNode()) {
            // If we are a client, simply update our knowledge of the current root
            heard_from_root = true;
            last_root_announcement = hardware->getMillis(); // Reset timeout
            updateCurrentRoot(claim->node_id);
        }
    } else if (header->type == (uint8_t)MeshPacketType::HEARTBEAT) {
        if (len < sizeof(HeartbeatPacket)) return;
        const HeartbeatPacket* heartbeat = (const HeartbeatPacket*)data;

        // Track network-wide BLE status from heartbeats
        if (heartbeat->has_ble_connection) {
            network_has_ble_root = true;
            memcpy(ble_root_mac, heartbeat->node_id, 6);
            last_ble_root_seen = hardware->getMillis();
            ESP_LOGD(TAG, "BLE root detected in network: %02X:%02X:%02X:%02X:%02X:%02X",
                     heartbeat->node_id[0], heartbeat->node_id[1], heartbeat->node_id[2],
                     heartbeat->node_id[3], heartbeat->node_id[4], heartbeat->node_id[5]);
        }

        // If we are the root, collect heartbeats to build network statistics
        if (isRootNode()) {
            updateNodeInfo(*heartbeat);
            ESP_LOGD(TAG, "Root received heartbeat from %02X:%02X:%02X:%02X:%02X:%02X, neighbors: %d, has_ble: %d",
                     heartbeat->node_id[0], heartbeat->node_id[1], heartbeat->node_id[2],
                     heartbeat->node_id[3], heartbeat->node_id[4], heartbeat->node_id[5],
                     heartbeat->neighbor_count, heartbeat->has_ble_connection);
        } else {
            // If heartbeat is from a root node, reset our root timeout
            if (heartbeat->is_root) {
                heard_from_root = true;
                last_root_announcement = hardware->getMillis();
            }
            ESP_LOGD(TAG, "Client received heartbeat from %02X:%02X:%02X:%02X:%02X:%02X, has_ble: %d",
                     heartbeat->node_id[0], heartbeat->node_id[1], heartbeat->node_id[2],
                     heartbeat->node_id[3], heartbeat->node_id[4], heartbeat->node_id[5],
                     heartbeat->has_ble_connection);
        }
    }
    // Handle other types later
}

// Helper to convert MAC to uint64 for map key
static uint64_t macToUint64(const uint8_t* mac) {
    uint64_t id = 0;
    for (int i = 0; i < 6; ++i) {
        id = (id << 8) | mac[i];
    }
    return id;
}


void ESPNowMeshCoordinator::cleanupNodeInfo() {
    uint32_t now = hardware->getMillis();
    if (now - this->last_node_cleanup < 10000) return; // Clean up every 10 seconds
    this->last_node_cleanup = now;

    for (auto it = this->known_nodes_info.begin(); it != this->known_nodes_info.end(); ) {
        if (now - it->second.last_seen > ESPNowMeshCoordinator::NODE_TIMEOUT_MS) {
            it = this->known_nodes_info.erase(it);
        } else {
            ++it;
        }
    }
    // Update total_nodes and avg_network_rssi after cleanup
    this->network_stats.total_nodes = this->known_nodes_info.size() + (this->isRootNode() ? 1 : 0); // +1 if self is root
    int32_t sum_rssi = 0;
    if (!this->known_nodes_info.empty()) {
        for (const auto& pair : this->known_nodes_info) {
            sum_rssi += pair.second.rssi;
        }
        this->network_stats.avg_network_rssi = sum_rssi / this->known_nodes_info.size();
    } else {
        this->network_stats.avg_network_rssi = 0;
    }
}

void ESPNowMeshCoordinator::updateNodeInfo(const HeartbeatPacket& hb) {
    uint64_t id = macToUint64(hb.node_id);
    uint32_t now = hardware->getMillis();

    NodeInfo info;
    memcpy(info.node_id, hb.node_id, 6);
    info.neighbor_count = hb.neighbor_count;
    info.rssi = hb.avg_rssi;
    info.last_seen = now;
    this->known_nodes_info[id] = info;

    this->cleanupNodeInfo(); // Clean up expired nodes after update
}

void ESPNowMeshCoordinator::forwardPacket(const void* packet, size_t len) {
    if (len > ESPNOW_MESH_MAX_PAYLOAD_LEN) return;
    
    // Create mutable copy
    uint8_t buffer[ESPNOW_MESH_MAX_PAYLOAD_LEN];
    memcpy(buffer, packet, len);
    
    PacketHeader* header = (PacketHeader*)buffer;
    if (header->ttl <= 1) return;
    
    header->ttl--;
    
    broadcastPacket(buffer, len);
}

uint32_t ESPNowMeshCoordinator::generatePacketId() {
    uint16_t node_part = node_id;
    // Initialize counter with random value to avoid collisions after rapid reboots
    static uint16_t counter = (uint16_t)(esp_random() & 0xFFFF);
    
    // 32-bit ID: 16-bit Node ID | 16-bit Counter
    // Redesign says: (node_suffix << 16) | (counter++)
    return ((uint32_t)node_part << 16) | (counter++);
}

void ESPNowMeshCoordinator::broadcastRootClaim(RootClaimReason reason) {
    ESP_LOGI(TAG, "Broadcasting root claim (Reason: %d)", (int)reason);

    // Update internal state based on the claim we are making
    current_claim_reason = reason;

    if (reason == RootClaimReason::BLE_CONNECTED) {
        if (current_role != NodeRole::MESH_ROOT_ACTIVE) {
            transitionToRole(NodeRole::MESH_ROOT_ACTIVE);
        }
    } else {
        // BUTTON_PRESS or BLE_DISCONNECTED -> autonomous root
        if (current_role != NodeRole::MESH_ROOT_AUTONOMOUS) {
            transitionToRole(NodeRole::MESH_ROOT_AUTONOMOUS);
        }
    }

    RootClaimPacket claim_packet;
    claim_packet.header.type = (uint8_t)MeshPacketType::ROOT_CLAIM;
    claim_packet.header.packet_id = generatePacketId();
    claim_packet.header.ttl = ESPNOW_MESH_DEFAULT_TTL;
    memcpy(claim_packet.node_id, local_mac, 6);
    claim_packet.reason = (uint8_t)reason;
    claim_packet.neighbor_count = getActiveNeighborCount();
    claim_packet.timestamp = hardware->getMillis();
    
    packet_tracker.markPacketSeen(claim_packet.header.packet_id); // Mark our own packet as seen
    broadcastPacket(&claim_packet, sizeof(RootClaimPacket));
}

bool ESPNowMeshCoordinator::shouldYieldTo(const RootClaimPacket& claim) {
    // If we are a client, we always update our root info (handled by caller usually)
    // But if caller asks if we should yield (implying we are root), check priority.
    if (!isRootNode()) return false; // Clients don't yield, they just follow

    RootClaimReason claim_reason = static_cast<RootClaimReason>(claim.reason);

    // Priority order: BLE_CONNECTED > BLE_DISCONNECTED > BUTTON_PRESS

    // BLE_CONNECTED always wins against everything except another BLE_CONNECTED
    if (claim_reason == RootClaimReason::BLE_CONNECTED) {
        if (current_claim_reason != RootClaimReason::BLE_CONNECTED) {
            return true;  // Non-BLE yields to BLE
        }
        // Both have BLE_CONNECTED - lower MAC wins (should be rare dual-BLE scenario)
        return memcmp(claim.node_id, local_mac, 6) < 0;
    }

    // BLE_DISCONNECTED beats BUTTON_PRESS
    if (claim_reason == RootClaimReason::BLE_DISCONNECTED) {
        if (current_claim_reason == RootClaimReason::BUTTON_PRESS) {
            return true;
        }
        // BLE_DISCONNECTED vs BLE_DISCONNECTED: neighbor count, then MAC
        if (current_claim_reason == RootClaimReason::BLE_DISCONNECTED) {
            if (claim.neighbor_count > getActiveNeighborCount()) {
                return true;
            }
            if (claim.neighbor_count == getActiveNeighborCount()) {
                return memcmp(claim.node_id, local_mac, 6) < 0;
            }
        }
        // BLE_DISCONNECTED does not beat BLE_CONNECTED (handled above)
        return false;
    }

    // BUTTON_PRESS - intentional user action to take control
    if (claim_reason == RootClaimReason::BUTTON_PRESS) {
        if (current_claim_reason == RootClaimReason::BUTTON_PRESS) {
            // Another node's user wants control - always yield to the newer claim
            // This is the expected behavior: pressing buttons = "I want control now"
            ESP_LOGI(TAG, "Yielding to newer BUTTON_PRESS claim (user intent takes priority)");
            return true;
        }
        // BUTTON_PRESS does not beat BLE_CONNECTED or BLE_DISCONNECTED
        return false;
    }

    return false;
}

void ESPNowMeshCoordinator::updateCurrentRoot(const uint8_t* root_mac) {
    memcpy(current_root_mac, root_mac, 6);
    ESP_LOGI(TAG, "Current root updated to: %02X:%02X:%02X:%02X:%02X:%02X",
             current_root_mac[0], current_root_mac[1], current_root_mac[2],
             current_root_mac[3], current_root_mac[4], current_root_mac[5]);
}

// Callbacks
void ESPNowMeshCoordinator::setPacketCallback(std::function<void(const GenericPacket&)> callback) {
    packet_callback = callback;
}

void ESPNowMeshCoordinator::setRoleChangeCallback(std::function<void(NodeRole, NodeRole)> callback) {
    role_change_callback = callback;
}

// Stats
const NetworkStats& ESPNowMeshCoordinator::getNetworkStats() const {
    return network_stats;
}

size_t ESPNowMeshCoordinator::getActiveNeighborCount() const {
    return neighbor_tracker ? neighbor_tracker->getNeighborCount() : 0;
}

size_t ESPNowMeshCoordinator::getReachableNodeCount() const {
    return network_stats.total_nodes; 
}

int8_t ESPNowMeshCoordinator::getAverageNeighborRSSI() const {
    return neighbor_tracker ? neighbor_tracker->getAverageRssi() : 0;
}

// Periodic check for network state (no automatic fallback election per redesign)
void ESPNowMeshCoordinator::checkForRootElection() {
    uint32_t current_time = hardware->getMillis();

    // Update neighbor tracker periodically
    if (neighbor_tracker) {
        neighbor_tracker->cleanup(); // Triggers cleanup of old neighbors
    }

    // Check if BLE root has disappeared
    if (network_has_ble_root && (current_time - last_ble_root_seen > MESH_ROOT_TIMEOUT_MS)) {
        ESP_LOGI(TAG, "BLE root timeout (%dms) - clearing network BLE status", MESH_ROOT_TIMEOUT_MS);
        network_has_ble_root = false;
        memset(ble_root_mac, 0, 6);
    }

    // If we are root, send heartbeats periodically
    if (isRootNode()) {
        heard_from_root = true; // We are the root
        last_root_announcement = current_time;

        if (current_time - last_heartbeat_sent > heartbeat_interval_ms) {
            sendHeartbeat();
            last_heartbeat_sent = current_time;
            // Randomize next heartbeat interval for staggering
            heartbeat_interval_ms = 10000 + (hardware->getRandom() % 2000); // 10-12 seconds
        }
        return;
    }

    // Check if we haven't heard from any root recently
    if (current_time - last_root_announcement > MESH_ROOT_TIMEOUT_MS) {
        heard_from_root = false;
    }

    // NO AUTOMATIC FALLBACK ELECTION per root election redesign
    // Nodes stay as MESH_CLIENT and run idle pattern locally until:
    // 1. BLE connects to this node
    // 2. User triggers button takeover
}


bool ESPNowMeshCoordinator::shouldAcceptBleConnection() const {
    // Check if we're in displacement cooldown period
    if (hardware->getMillis() < displaced_until) {
        ESP_LOGW(TAG, "BLE connection rejected - in displacement cooldown period");
        return false;
    }
    return true;
}

void ESPNowMeshCoordinator::sendHeartbeat() {
    HeartbeatPacket hb;
    hb.header.type = (uint8_t)MeshPacketType::HEARTBEAT;
    hb.header.packet_id = generatePacketId();
    hb.header.ttl = ESPNOW_MESH_DEFAULT_TTL;
    memcpy(hb.node_id, local_mac, 6);
    hb.neighbor_count = getActiveNeighborCount();
    hb.avg_rssi = getAverageNeighborRSSI();
    hb.uptime_seconds = hardware->getMillis() / 1000;
    hb.is_root = isRootNode() ? 1 : 0;
    hb.has_ble_connection = ble_connected ? 1 : 0;

    packet_tracker.markPacketSeen(hb.header.packet_id);
    broadcastPacket(&hb, sizeof(HeartbeatPacket));

    ESP_LOGD(TAG, "Sent heartbeat (neighbors: %d, RSSI: %d, uptime: %lu, has_ble: %d)",
             hb.neighbor_count, hb.avg_rssi, hb.uptime_seconds, hb.has_ble_connection);
}

void ESPNowMeshCoordinator::randomBackoff() {
    // Not strictly needed for simple broadcast if using esp_now directly, 
    // but good practice.
    vTaskDelay(pdMS_TO_TICKS(5 + (hardware->getRandom() % 10)));
}



// New methods for root election redesign

bool ESPNowMeshCoordinator::networkHasBleRoot() const {
    // Check if we've seen a BLE root recently
    // This accounts for packet loss and stale information
    if (!network_has_ble_root) return false;
    uint32_t now = hardware->getMillis();
    return (now - last_ble_root_seen) < MESH_BLE_ROOT_TIMEOUT_MS;
}

bool ESPNowMeshCoordinator::hasActiveRoot() const {
    // Either we are the root, or we've heard from one recently
    if (isRootNode()) return true;
    uint32_t now = hardware->getMillis();
    return heard_from_root && (now - last_root_announcement < MESH_ROOT_TIMEOUT_MS);
}

void ESPNowMeshCoordinator::setBleDisplacementCallback(std::function<void()> callback) {
    ble_displacement_callback = callback;
}

void ESPNowMeshCoordinator::handleBleDisplacement(const RootClaimPacket& claim) {
    ESP_LOGW(TAG, "BLE displacement: another node (%02X:%02X:%02X:%02X:%02X:%02X) claimed BLE root",
             claim.node_id[0], claim.node_id[1], claim.node_id[2],
             claim.node_id[3], claim.node_id[4], claim.node_id[5]);

    // Set cooldown period to prevent immediate reconnection
    displaced_until = hardware->getMillis() + MESH_DISPLACEMENT_COOLDOWN_MS;

    // Notify callback (will trigger phone notification + disconnect)
    if (ble_displacement_callback) {
        ble_displacement_callback();
    }
}
