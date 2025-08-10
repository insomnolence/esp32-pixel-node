#include "espnow_mesh_coordinator.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_mac.h"
#include <string.h>
#include <ctime>

// Adaptive mesh components
#include "adaptive/neighbor_manager.h"
#include "adaptive/topology_manager.h"
#include "adaptive/router.h"

// BLE debugging
#include "bluetooth/ble_gatt_server.h"

const char* ESPNowMeshCoordinator::TAG = "ESPNowMeshCoordinator";
ESPNowMeshCoordinator* ESPNowMeshCoordinator::instance = nullptr;
SemaphoreHandle_t ESPNowMeshCoordinator::instance_mutex = nullptr;

// BoundedPacketTracker implementation
BoundedPacketTracker::BoundedPacketTracker() 
    : history_index(0)
    , history_count(0)
    , last_cleanup(0)
{
    // Initialize packet history to zero
    memset(packet_history, 0, sizeof(packet_history));
}

bool BoundedPacketTracker::isPacketSeen(uint32_t packet_id) {
    uint32_t now = esp_timer_get_time() / 1000; // Convert to ms
    
    // Periodic cleanup (less frequent with bounded buffer)
    if (now - last_cleanup > CLEANUP_INTERVAL_MS) {
        cleanup();
        last_cleanup = now;
    }
    
    // Search through recent packet history
    for (size_t i = 0; i < history_count; ++i) {
        if (packet_history[i] == packet_id) {
            return true; // Already seen
        }
    }
    
    // Add to circular buffer
    packet_history[history_index] = packet_id;
    history_index = (history_index + 1) % PACKET_HISTORY_SIZE;
    
    if (history_count < PACKET_HISTORY_SIZE) {
        history_count++;
    }
    
    return false; // New packet
}

void BoundedPacketTracker::cleanup() {
    // With circular buffer, no cleanup needed - oldest entries are automatically overwritten
    // This method exists for compatibility and potential future use
    ESP_LOGD("BoundedPacketTracker", "Packet tracker maintenance complete (circular buffer)");
}

// ESPNowMeshCoordinator implementation
ESPNowMeshCoordinator::ESPNowMeshCoordinator() 
    : current_role(NodeRole::MESH_CLIENT)
    , node_id(0)
    , ble_connected(false)
    , packet_counter(0)
    , mesh_task_handle(nullptr)
    , election_timer(0)
    , last_root_announcement(0)
    , heard_from_root(false)
    , ble_connection_uptime_ms(0)
    , has_ble_connection_timestamp(false)
    , election_state(ElectionState::ELECTION_IDLE)
    , election_start_time(0)
    , election_phase_timeout(0)
    , election_candidate_count(0)
    , elected_root_node_id(0)
    , last_election_attempt(0)
    , autonomous_root_timestamp(0)
    , adaptive_mesh_enabled(false)
{
    // Create mutex for thread-safe singleton access if not already created
    if (instance_mutex == nullptr) {
        instance_mutex = xSemaphoreCreateMutex();
        if (instance_mutex == nullptr) {
            ESP_LOGE(TAG, "Failed to create instance mutex");
        }
    }
    
    instance = this;
    memset(local_mac, 0, 6);
    memset(election_candidates, 0, sizeof(election_candidates));
}

ESPNowMeshCoordinator::~ESPNowMeshCoordinator() {
    stop();
    instance = nullptr;
    
    // Clean up mutex when last instance is destroyed
    if (instance_mutex != nullptr) {
        vSemaphoreDelete(instance_mutex);
        instance_mutex = nullptr;
    }
}

esp_err_t ESPNowMeshCoordinator::init() {
    ESP_LOGI(TAG, "Initializing ESP-NOW Mesh Coordinator");
    
    // Initialize WiFi first
    esp_err_t ret = initWiFi();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize ESP-NOW
    ret = initESPNow();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Get local MAC address for node ID
    esp_wifi_get_mac(WIFI_IF_STA, local_mac);
    node_id = (local_mac[4] << 8) | local_mac[5]; // Use last 2 bytes as node ID
    
    ESP_LOGI(TAG, "ESP-NOW Mesh initialized - Node ID: 0x%04X", node_id);
    ESP_LOGI(TAG, "MAC Address: %02x:%02x:%02x:%02x:%02x:%02x", 
             local_mac[0], local_mac[1], local_mac[2], 
             local_mac[3], local_mac[4], local_mac[5]);
    
    return ESP_OK;
}

esp_err_t ESPNowMeshCoordinator::initWiFi() {
    // Initialize netif and event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create WiFi station netif
    esp_netif_create_default_wifi_sta();
    
    // Initialize WiFi with coexistence support
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Set WiFi mode to APSTA for better BLE coexistence
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    
    // Start WiFi first
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Set channel for ESP-NOW (must be done after WiFi start)
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_MESH_CHANNEL, WIFI_SECOND_CHAN_NONE));
    
    ESP_LOGI(TAG, "WiFi initialized for ESP-NOW mesh on channel %d", ESPNOW_MESH_CHANNEL);
    return ESP_OK;
}

esp_err_t ESPNowMeshCoordinator::initESPNow() {
    // Initialize ESP-NOW
    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register callbacks
    ESP_ERROR_CHECK(esp_now_register_send_cb(onESPNowSent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(onESPNowReceived));
    
    // Add broadcast peer for mesh communication
    esp_now_peer_info_t broadcast_peer = {};
    memset(broadcast_peer.peer_addr, 0xFF, 6); // Broadcast MAC
    broadcast_peer.channel = ESPNOW_MESH_CHANNEL;
    broadcast_peer.encrypt = false;
    
    ret = esp_now_add_peer(&broadcast_peer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "ESP-NOW initialized with broadcast peer");
    return ESP_OK;
}

esp_err_t ESPNowMeshCoordinator::start() {
    ESP_LOGI(TAG, "Starting ESP-NOW mesh coordinator");
    
    // ESP-NOW is already started during init
    // No separate mesh task needed - callbacks handle everything
    
    // Start autonomous root election timer with node-specific randomization to prevent conflicts
    uint32_t base_delay = 15000; // 15 second base delay
    uint32_t random_delay = esp_random() % 30000; // 0-30 seconds random
    uint32_t node_offset = (node_id % 1000) * 10; // Node-specific offset (0-9.99 seconds)
    uint32_t election_delay = base_delay + random_delay + node_offset; // 15-54.99 seconds total
    election_timer = esp_timer_get_time() / 1000 + election_delay;
    
    ESP_LOGI(TAG, "ESP-NOW mesh coordinator started successfully");
    ESP_LOGI(TAG, "Autonomous root election will start in %lu seconds", election_delay / 1000);
    return ESP_OK;
}

esp_err_t ESPNowMeshCoordinator::stop() {
    ESP_LOGI(TAG, "Stopping ESP-NOW mesh coordinator");
    
    if (mesh_task_handle) {
        vTaskDelete(mesh_task_handle);
        mesh_task_handle = nullptr;
    }
    
    esp_now_deinit();
    return ESP_OK;
}

void ESPNowMeshCoordinator::onBleConnected() {
    ESP_LOGI(TAG, "🔥 BLE connected - becoming mesh root with mobile control (Node 0x%04X)", node_id);
    ESP_LOGI(TAG, "🔥 Previous role: %s", getRoleString());
    
    ble_connected = true;
    
    // Record BLE connection timestamp for priority comparison
    ble_connection_uptime_ms = esp_timer_get_time() / 1000;
    has_ble_connection_timestamp = true;
    
    ESP_LOGI(TAG, "🔥 BLE connection timestamp recorded: uptime %lu ms", ble_connection_uptime_ms);
    
    // BLE connection always takes priority over autonomous root
    if (current_role == NodeRole::MESH_ROOT_AUTONOMOUS) {
        ESP_LOGI(TAG, "🔥 Upgrading from autonomous root to BLE-controlled root");
    }
    
    ESP_LOGI(TAG, "🔥 Transitioning to MESH_ROOT_ACTIVE role");
    transitionToRole(NodeRole::MESH_ROOT_ACTIVE);
    ESP_LOGI(TAG, "🔥 Role transition complete: %s", getRoleString());
    
    // Immediately announce BLE root status to make other autonomous roots step down
    ESP_LOGI(TAG, "🔥 Sending immediate BLE root announcement to mesh");
    sendRootAnnouncement();
    ESP_LOGI(TAG, "🔥 BLE root announcement sent - other autonomous roots should step down");
}

void ESPNowMeshCoordinator::onBleDisconnected() {
    ESP_LOGI(TAG, "BLE disconnected");
    ble_connected = false;
    has_ble_connection_timestamp = false;  // Clear timestamp validity
    
    // If we were BLE root, step down but may become autonomous root later
    if (current_role == NodeRole::MESH_ROOT_ACTIVE) {
        ESP_LOGI(TAG, "Stepping down from BLE root - may become autonomous root");
        transitionToRole(NodeRole::MESH_CLIENT);
        
        // Reset election timer for potential autonomous root election
        election_timer = esp_timer_get_time() / 1000 + 10000; // 10 seconds
    }
}

esp_err_t ESPNowMeshCoordinator::transitionToRole(NodeRole new_role) {
    NodeRole old_role = current_role;
    
    if (old_role == new_role) {
        return ESP_OK; // No change needed
    }
    
    ESP_LOGI(TAG, "Role transition: %d -> %d", (int)old_role, (int)new_role);
    current_role = new_role;
    
    // Track when we become autonomous root for BLE stabilization
    if (new_role == NodeRole::MESH_ROOT_AUTONOMOUS) {
        autonomous_root_timestamp = esp_timer_get_time() / 1000;
        ESP_LOGI(TAG, "🔒 Autonomous root stabilization period started - BLE connections delayed for 3 seconds");
    }
    
    // Notify callback if set
    if (role_change_callback) {
        role_change_callback(old_role, new_role);
    }
    
    return ESP_OK;
}

esp_err_t ESPNowMeshCoordinator::sendGenericPacket(const GenericPacket& packet) {
    return sendLEDPattern(packet); // All packets are high priority for LED sync
}

esp_err_t ESPNowMeshCoordinator::sendLEDPattern(const GenericPacket& pattern) {
    if (!pattern.isValid()) {
        ESP_LOGE(TAG, "Cannot send invalid LED pattern");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create mesh packet for LED pattern (high priority, full TTL)
    ESPNowMeshPacket mesh_packet = createMeshPacket(MeshPacketType::LED_PATTERN, pattern, ESPNOW_MESH_DEFAULT_TTL);
    
    ESP_LOGI(TAG, "Sending LED pattern HIGH PRIORITY (ID: 0x%08lX, size: %zu bytes)", 
             mesh_packet.packet_id, pattern.getLength());
    
    return sendMeshPacketHighPriority(mesh_packet);
}

ESPNowMeshPacket ESPNowMeshCoordinator::createMeshPacket(MeshPacketType type, const GenericPacket& payload, uint8_t ttl) {
    ESPNowMeshPacket packet = {};
    
    packet.packet_id = generatePacketId();
    packet.packet_type = (uint8_t)type;
    packet.ttl = ttl;
    memcpy(packet.source_mac, local_mac, 6);
    packet.timestamp = esp_timer_get_time() / 1000; // Convert to ms
    packet.data_len = std::min((size_t)(ESPNOW_MESH_MAX_PAYLOAD_LEN - 20), payload.getLength());
    
    // Copy payload data
    memcpy(packet.data, payload.getData(), packet.data_len);
    
    // Mark packet as seen to prevent loop-back
    packet_tracker.isPacketSeen(packet.packet_id);
    
    return packet;
}

uint32_t ESPNowMeshCoordinator::generatePacketId() {
    // Combine node ID, packet counter, and timestamp for unique ID
    uint32_t timestamp = esp_timer_get_time() / 1000;
    return (node_id << 16) | ((packet_counter++ & 0xFFFF) ^ (timestamp & 0xFFFF));
}

esp_err_t ESPNowMeshCoordinator::sendMeshPacket(const ESPNowMeshPacket& packet) {
    return sendMeshPacketWithRetry(packet, 3);
}

esp_err_t ESPNowMeshCoordinator::sendMeshPacketWithRetry(const ESPNowMeshPacket& packet, int max_retries) {
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    size_t packet_size = sizeof(ESPNowMeshPacket) - sizeof(packet.data) + packet.data_len;
    
    for (int attempt = 0; attempt < max_retries; attempt++) {
        // Random backoff to prevent collisions
        randomBackoff();
        
        esp_err_t result = esp_now_send(broadcast_mac, (uint8_t*)&packet, packet_size);
        
        if (result == ESP_OK) {
            network_stats.packets_sent++;
            return ESP_OK;
        }
        
        network_stats.send_failures++;
        ESP_LOGW(TAG, "Send attempt %d/%d failed: %s", attempt + 1, max_retries, esp_err_to_name(result));
        
        // Exponential backoff for retries
        if (attempt < max_retries - 1) {
            vTaskDelay(pdMS_TO_TICKS(10 * (attempt + 1)));
        }
    }
    
    ESP_LOGE(TAG, "Failed to send mesh packet after %d attempts", max_retries);
    return ESP_FAIL;
}

esp_err_t ESPNowMeshCoordinator::sendMeshPacketHighPriority(const ESPNowMeshPacket& packet) {
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    size_t packet_size = sizeof(ESPNowMeshPacket) - sizeof(packet.data) + packet.data_len;
    
    // HIGH PRIORITY: Send immediately without backoff, single attempt
    esp_err_t result = esp_now_send(broadcast_mac, (uint8_t*)&packet, packet_size);
    
    if (result == ESP_OK) {
        network_stats.packets_sent++;
        ESP_LOGD(TAG, "High priority packet sent successfully");
        return ESP_OK;
    }
    
    network_stats.send_failures++;
    ESP_LOGW(TAG, "High priority send failed: %s", esp_err_to_name(result));
    
    // Single retry with minimal delay for critical LED sync
    vTaskDelay(pdMS_TO_TICKS(2)); // 2ms delay only
    result = esp_now_send(broadcast_mac, (uint8_t*)&packet, packet_size);
    
    if (result == ESP_OK) {
        network_stats.packets_sent++;
        ESP_LOGD(TAG, "High priority packet sent on retry");
        return ESP_OK;
    }
    
    network_stats.send_failures++;
    ESP_LOGE(TAG, "High priority send failed on retry: %s", esp_err_to_name(result));
    return ESP_FAIL;
}

esp_err_t ESPNowMeshCoordinator::sendMeshPacketToNode(const ESPNowMeshPacket& packet, uint16_t target_node_id) {
    if (!adaptive_mesh_enabled || !neighbor_manager) {
        ESP_LOGW(TAG, "Adaptive mesh not enabled - falling back to broadcast");
        return sendMeshPacket(packet);
    }
    
    // Find the target node in our neighbor table
    const NeighborInfo* target_neighbor = neighbor_manager->getNeighborByNodeId(target_node_id);
    if (!target_neighbor) {
        ESP_LOGW(TAG, "Target node 0x%04X not in neighbor table - falling back to broadcast", target_node_id);
        return sendMeshPacket(packet);
    }
    
    // Add target as ESP-NOW peer if not already added
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, target_neighbor->mac_addr, 6);
    peer_info.channel = ESPNOW_MESH_CHANNEL;
    peer_info.encrypt = false;
    
    esp_err_t peer_result = esp_now_add_peer(&peer_info);
    if (peer_result != ESP_OK && peer_result != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW(TAG, "Failed to add peer for node 0x%04X: %s", target_node_id, esp_err_to_name(peer_result));
        return sendMeshPacket(packet); // Fall back to broadcast
    }
    
    // Send directly to target node
    size_t packet_size = sizeof(ESPNowMeshPacket) - sizeof(packet.data) + packet.data_len;
    esp_err_t result = esp_now_send(target_neighbor->mac_addr, (uint8_t*)&packet, packet_size);
    
    if (result == ESP_OK) {
        network_stats.packets_sent++;
        ESP_LOGI(TAG, "🎯 Unicast packet sent to node 0x%04X (" MACSTR ")", 
                 target_node_id, MAC2STR(target_neighbor->mac_addr));
        return ESP_OK;
    } else {
        network_stats.send_failures++;
        ESP_LOGW(TAG, "❌ Unicast send to node 0x%04X failed: %s", 
                 target_node_id, esp_err_to_name(result));
        return result;
    }
}

esp_err_t ESPNowMeshCoordinator::sendMeshPacketToAllNeighbors(const ESPNowMeshPacket& packet) {
    if (!adaptive_mesh_enabled || !neighbor_manager) {
        ESP_LOGW(TAG, "Adaptive mesh not enabled - falling back to broadcast");
        return sendMeshPacket(packet);
    }
    
    size_t active_neighbors = neighbor_manager->getActiveNeighborCount();
    if (active_neighbors == 0) {
        ESP_LOGD(TAG, "No active neighbors - skipping intelligent flooding");
        return ESP_OK;
    }
    
    // Get all neighbors
    std::array<NeighborInfo, MAX_NEIGHBORS> neighbors;
    size_t neighbor_count = neighbor_manager->getAllNeighbors(neighbors);
    
    if (neighbor_count == 0) {
        ESP_LOGD(TAG, "No neighbors available for intelligent flooding");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "🌊 Intelligent flooding to %zu neighbors", neighbor_count);
    
    size_t successful_sends = 0;
    for (size_t i = 0; i < neighbor_count; ++i) {
        const NeighborInfo& neighbor = neighbors[i];
        
        // Skip neighbors with poor link quality for flooding
        if (neighbor.link_quality == LinkQuality::UNUSABLE) {
            continue;
        }
        
        // Add neighbor as ESP-NOW peer if not already added
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, neighbor.mac_addr, 6);
        peer_info.channel = ESPNOW_MESH_CHANNEL;
        peer_info.encrypt = false;
        
        esp_err_t peer_result = esp_now_add_peer(&peer_info);
        if (peer_result != ESP_OK && peer_result != ESP_ERR_ESPNOW_EXIST) {
            ESP_LOGD(TAG, "Could not add peer for node 0x%04X", neighbor.node_id);
            continue;
        }
        
        // Send to this neighbor
        size_t packet_size = sizeof(ESPNowMeshPacket) - sizeof(packet.data) + packet.data_len;
        esp_err_t result = esp_now_send(neighbor.mac_addr, (uint8_t*)&packet, packet_size);
        
        if (result == ESP_OK) {
            successful_sends++;
            ESP_LOGV(TAG, "Flooded to node 0x%04X (" MACSTR ")", 
                     neighbor.node_id, MAC2STR(neighbor.mac_addr));
        } else {
            ESP_LOGD(TAG, "Failed to flood to node 0x%04X: %s", 
                     neighbor.node_id, esp_err_to_name(result));
        }
        
        // Small delay between sends to prevent congestion
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    network_stats.packets_sent += successful_sends;
    ESP_LOGI(TAG, "🌊 Intelligent flooding complete: %zu/%zu successful", 
             successful_sends, neighbor_count);
    
    return (successful_sends > 0) ? ESP_OK : ESP_FAIL;
}

void ESPNowMeshCoordinator::randomBackoff() {
    // Adaptive random delay based on node ID to reduce collisions
    // Range: 5-25ms with node-specific offset
    uint32_t base_delay = 5 + (esp_random() % 21); // 5-25ms base
    uint32_t node_offset = (node_id % 10); // 0-9ms node-specific offset
    uint32_t total_delay = base_delay + node_offset;
    
    if (total_delay > 0) {
        vTaskDelay(pdMS_TO_TICKS(total_delay));
    }
}

void ESPNowMeshCoordinator::onESPNowSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Thread-safe access to singleton instance from interrupt context
    if (instance_mutex != nullptr && xSemaphoreTake(instance_mutex, 0) == pdTRUE) {
        if (instance) {
            ESP_LOGD(instance->TAG, "ESP-NOW send status: %s", 
                     status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
        }
        xSemaphoreGive(instance_mutex);
    }
}

void ESPNowMeshCoordinator::onESPNowReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    // Thread-safe access to singleton instance from interrupt context
    if (instance_mutex != nullptr && xSemaphoreTake(instance_mutex, 0) == pdTRUE) {
        if (instance) {
            instance->handleReceivedPacket(recv_info->src_addr, data, len);
        }
        xSemaphoreGive(instance_mutex);
    }
}

void ESPNowMeshCoordinator::handleReceivedPacket(const uint8_t *mac_addr, const uint8_t *data, int len) {
    const size_t min_packet_size = sizeof(ESPNowMeshPacket) - ESPNOW_MESH_MAX_PAYLOAD_LEN + 20;
    if (len < min_packet_size) {
        ESP_LOGW(TAG, "Packet too small: %d < %zu", len, min_packet_size);
        network_stats.packets_dropped++;
        return;
    }
    
    ESPNowMeshPacket* mesh_packet = (ESPNowMeshPacket*)data;
    
    // CRITICAL: Validate data_len before using it
    const size_t max_data_len = len - min_packet_size;
    if (mesh_packet->data_len > max_data_len) {
        ESP_LOGE(TAG, "Invalid data_len: %d > %zu", mesh_packet->data_len, max_data_len);
        network_stats.packets_dropped++;
        return;
    }
    
    // Check if we've seen this packet before (prevent loops)
    if (packet_tracker.isPacketSeen(mesh_packet->packet_id)) {
        ESP_LOGD(TAG, "Dropping duplicate packet ID: 0x%08lX", mesh_packet->packet_id);
        network_stats.packets_dropped++;
        return;
    }
    
    // Update network statistics
    network_stats.packets_received++;
    network_stats.last_activity_ms = esp_timer_get_time() / 1000;
    
    // Check if packet came from ourselves (additional loop prevention)
    if (memcmp(mesh_packet->source_mac, local_mac, 6) == 0) {
        ESP_LOGD(TAG, "Dropping packet from self");
        return;
    }
    
    ESP_LOGI(TAG, "Received mesh packet: ID=0x%08lX, type=%d, TTL=%d, size=%d", 
             mesh_packet->packet_id, mesh_packet->packet_type, mesh_packet->ttl, mesh_packet->data_len);
    
    // Process packet based on type
    switch ((MeshPacketType)mesh_packet->packet_type) {
        case MeshPacketType::LED_PATTERN:
            // Forward immediately, then process locally
            forwardPacket(*mesh_packet);
            
            // Process LED pattern locally
            if (packet_callback && mesh_packet->data_len > 0) {
                GenericPacket led_pattern(mesh_packet->data, mesh_packet->data_len);
                if (led_pattern.isValid()) {
                    ESP_LOGI(TAG, "Processing LED pattern locally");
                    packet_callback(led_pattern);
                } else {
                    ESP_LOGW(TAG, "Received invalid LED pattern");
                }
            }
            break;
            
        case MeshPacketType::NODE_STATUS:
            // Forward and log status
            forwardPacket(*mesh_packet);
            ESP_LOGI(TAG, "Node status update received");
            break;
            
        case MeshPacketType::NETWORK_BEACON:
            // Process beacon (future use for network discovery)
            ESP_LOGD(TAG, "Network beacon received");
            break;
            
        case MeshPacketType::ROOT_ELECTION:
            ESP_LOGI(TAG, "Legacy root election packet received from node 0x%04X", 
                     (mesh_packet->source_mac[4] << 8) | mesh_packet->source_mac[5]);
            // Reset our election timer if someone else is trying to become root
            election_timer = esp_timer_get_time() / 1000 + 15000; // Wait 15 more seconds
            break;
            
        case MeshPacketType::ELECTION_DISCOVERY:
        case MeshPacketType::ELECTION_CANDIDATE:
        case MeshPacketType::ELECTION_VOTE:
        case MeshPacketType::ELECTION_RESULT:
            // CRITICAL: Forward election packets for multi-hop election coordination
            forwardPacket(*mesh_packet);
            
            // Process advanced election packets
            processElectionPacket(*mesh_packet);
            break;
            
        case MeshPacketType::ROOT_ANNOUNCEMENT:
            ESP_LOGI(TAG, "Root announcement received from node 0x%04X", 
                     (mesh_packet->source_mac[4] << 8) | mesh_packet->source_mac[5]);
            
            // CRITICAL: Forward root announcements to ensure all nodes in multi-hop network receive them
            forwardPacket(*mesh_packet);
            
            heard_from_root = true;
            last_root_announcement = esp_timer_get_time() / 1000;
            ESP_LOGD(TAG, "Updated last_root_announcement to %lu ms (heard_from_root=true)", last_root_announcement);
            
            // Process root announcement with BLE priority comparison
            if (mesh_packet->data_len >= 8) {
                bool announcing_node_has_ble = (mesh_packet->data[6] == 1); // BLE status flag
                uint16_t announcing_node_id = (mesh_packet->data[4] << 8) | mesh_packet->data[5];
                
                // Extract BLE connection age if available (12-byte format)
                uint32_t their_ble_age = UINT32_MAX;  // Default to oldest (lowest priority)
                if (mesh_packet->data_len >= 12) {
                    their_ble_age = ((uint32_t)mesh_packet->data[8] << 24) |
                                   ((uint32_t)mesh_packet->data[9] << 16) |
                                   ((uint32_t)mesh_packet->data[10] << 8) |
                                   ((uint32_t)mesh_packet->data[11]);
                }
                
                if (announcing_node_has_ble) {
                    ESP_LOGI(TAG, "Detected BLE-connected root announcement from node 0x%04X (BLE age: %lu ms)", 
                             announcing_node_id, their_ble_age);
                    
                    // If we're autonomous root, step down for any BLE root (higher priority)
                    if (current_role == NodeRole::MESH_ROOT_AUTONOMOUS) {
                        ESP_LOGI(TAG, "Stepping down from autonomous root - BLE root detected (node 0x%04X)", announcing_node_id);
                        transitionToRole(NodeRole::MESH_CLIENT);
                        election_timer = esp_timer_get_time() / 1000 + 60000; // Wait 60 seconds before next election attempt
                    }
                    // NEW: BLE-to-BLE priority comparison - newer BLE connection wins
                    else if (current_role == NodeRole::MESH_ROOT_ACTIVE && ble_connected) {
                        uint32_t our_ble_age = getBleConnectionAge();
                        
                        ESP_LOGI(TAG, "BLE root priority comparison: Our BLE age %lu ms vs Their BLE age %lu ms", 
                                 our_ble_age, their_ble_age);
                        
                        bool they_have_newer_ble = false;
                        
                        if (their_ble_age < our_ble_age) {
                            // They connected more recently
                            they_have_newer_ble = true;
                        } else if (their_ble_age == our_ble_age) {
                            // Same connection age, use node ID as tiebreaker (lower node ID wins)
                            they_have_newer_ble = (announcing_node_id < node_id);
                        }
                        
                        if (they_have_newer_ble) {
                            ESP_LOGI(TAG, "Stepping down from BLE root - Node 0x%04X has newer BLE connection (%lu vs %lu ms ago)", 
                                     announcing_node_id, their_ble_age, our_ble_age);
                            transitionToRole(NodeRole::MESH_CLIENT);
                            election_timer = esp_timer_get_time() / 1000 + 60000; // Wait 60 seconds before next election attempt
                        } else {
                            ESP_LOGI(TAG, "Maintaining BLE root - Our BLE connection is newer or equal priority");
                        }
                    }
                } else {
                    ESP_LOGI(TAG, "Received autonomous root announcement from node 0x%04X", announcing_node_id);
                    
                    // CRITICAL: Autonomous-to-autonomous conflict resolution  
                    // If we're both autonomous roots, the one with higher priority (lower node ID) should win
                    if (current_role == NodeRole::MESH_ROOT_AUTONOMOUS) {
                        if (announcing_node_id < node_id) {
                            ESP_LOGI(TAG, "🚨 Autonomous root conflict: Node 0x%04X has higher priority than us (0x%04X)", 
                                     announcing_node_id, node_id);
                            ESP_LOGI(TAG, "🚨 Stepping down from autonomous root - Node 0x%04X takes precedence", announcing_node_id);
                            transitionToRole(NodeRole::MESH_CLIENT);
                            election_timer = esp_timer_get_time() / 1000 + 60000; // Wait 60 seconds before next election attempt
                        } else {
                            ESP_LOGI(TAG, "🏆 Maintaining autonomous root - Our priority (0x%04X) is higher than Node 0x%04X", 
                                     node_id, announcing_node_id);
                        }
                    }
                }
            }
            
            // Cancel our own election if we were about to become root
            if (current_role == NodeRole::MESH_CLIENT) {
                election_timer = esp_timer_get_time() / 1000 + 45000; // Wait 45 seconds before next attempt
            }
            break;
            
        // Adaptive Mesh Packet Processing
        case MeshPacketType::ADAPTIVE_NEIGHBOR_DISCOVERY:
            if (adaptive_mesh_enabled && neighbor_manager) {
                processAdaptiveNeighborDiscovery(*mesh_packet);
            }
            break;
            
        case MeshPacketType::ADAPTIVE_TOPOLOGY_UPDATE:
            if (adaptive_mesh_enabled && topology_manager) {
                processAdaptiveTopologyUpdate(*mesh_packet);
            }
            break;
            
        case MeshPacketType::ADAPTIVE_ROUTE_REQUEST:
        case MeshPacketType::ADAPTIVE_ROUTE_REPLY:
            if (adaptive_mesh_enabled && adaptive_router) {
                processAdaptiveRouting(*mesh_packet);
            }
            break;
            
        case MeshPacketType::ADAPTIVE_DATA_FORWARD:
            if (adaptive_mesh_enabled && adaptive_router) {
                processAdaptiveDataForward(*mesh_packet);
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown packet type: %d", mesh_packet->packet_type);
            break;
    }
}

void ESPNowMeshCoordinator::forwardPacket(const ESPNowMeshPacket& packet) {
    if (packet.ttl <= 1) {
        ESP_LOGD(TAG, "Packet TTL expired, not forwarding");
        return;
    }
    
    // Create forwarded packet with decremented TTL
    ESPNowMeshPacket forward_packet = packet;
    forward_packet.ttl--;
    
    ESP_LOGD(TAG, "Forwarding packet ID: 0x%08lX (TTL: %d -> %d)", 
             packet.packet_id, packet.ttl, forward_packet.ttl);
    
    // Use adaptive routing if enabled, otherwise fall back to broadcast
    if (adaptive_mesh_enabled && adaptive_router) {
        // Get destination node from source MAC (2 byte node ID)
        uint16_t destination_node = (packet.source_mac[4] << 8) | packet.source_mac[5];
        
        // For broadcast packets (LED patterns), use broadcast routing
        if (packet.packet_type == static_cast<uint8_t>(MeshPacketType::LED_PATTERN)) {
            // LED patterns should reach all nodes - use intelligent flooding
            ESP_LOGD(TAG, "🎨 Adaptive forwarding LED pattern to all reachable nodes");
            sendMeshPacketToAllNeighbors(forward_packet);
        } else {
            // For unicast packets, use adaptive routing
            uint16_t next_hop = adaptive_router->selectBestNextHop(destination_node, PacketPriority::NORMAL);
            if (next_hop != 0) {
                ESP_LOGD(TAG, "🛣️ Adaptive routing to node 0x%04X via next hop 0x%04X", destination_node, next_hop);
                sendMeshPacketToNode(forward_packet, next_hop);
            } else {
                ESP_LOGD(TAG, "❌ No route found to node 0x%04X, dropping packet", destination_node);
                network_stats.packets_dropped++;
                return;
            }
        }
    } else {
        // Fall back to flat broadcast
        ESP_LOGD(TAG, "📡 Flat broadcast forwarding (adaptive mesh disabled)");
        sendMeshPacket(forward_packet);
    }
}

// Getter methods
NodeRole ESPNowMeshCoordinator::getCurrentRole() const { 
    return current_role; 
}

uint16_t ESPNowMeshCoordinator::getNodeId() const { 
    return node_id; 
}

const char* ESPNowMeshCoordinator::getRoleString() const {
    switch (current_role) {
        case NodeRole::MESH_ROOT_ACTIVE:
            return "ROOT (Mobile connected)";
        case NodeRole::MESH_ROOT_AUTONOMOUS:
            return "ROOT (Autonomous - no mobile)";
        case NodeRole::MESH_CLIENT:
        default:
            return "CLIENT (Receiving patterns)";
    }
}

bool ESPNowMeshCoordinator::isRootNode() const { 
    return current_role == NodeRole::MESH_ROOT_ACTIVE || current_role == NodeRole::MESH_ROOT_AUTONOMOUS; 
}

const ESPNowMeshCoordinator::NetworkStats& ESPNowMeshCoordinator::getNetworkStats() const {
    return network_stats;
}

// Callback setters
void ESPNowMeshCoordinator::setPacketCallback(std::function<void(const GenericPacket&)> callback) {
    packet_callback = callback;
}

void ESPNowMeshCoordinator::setRoleChangeCallback(std::function<void(NodeRole, NodeRole)> callback) {
    role_change_callback = callback;
}

// Autonomous root election implementation
void ESPNowMeshCoordinator::startRootElection() {
    ESP_LOGI(TAG, "Starting autonomous root election");
    election_timer = esp_timer_get_time() / 1000 + (5000 + (node_id % 10000)); // 5-15 second delay based on node ID
}

bool ESPNowMeshCoordinator::isAutonomousRoot() const {
    return current_role == NodeRole::MESH_ROOT_AUTONOMOUS;
}

void ESPNowMeshCoordinator::checkForRootElection() {
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Run adaptive mesh neighbor discovery if enabled
    if (adaptive_mesh_enabled && neighbor_manager) {
        neighbor_manager->startNeighborDiscovery();
    }
    
    // Only clients can participate in autonomous root election
    if (current_role != NodeRole::MESH_CLIENT) {
        return;
    }
    
    // BLE-connected nodes should not start autonomous root elections
    if (ble_connected) {
        ESP_LOGD(TAG, "Skipping root election - node has BLE connection");
        return;
    }
    
    // Check if we haven't heard from any root recently
    if (current_time - last_root_announcement > 30000) { // 30 seconds without root
        ESP_LOGD(TAG, "Root timeout: current=%lu, last_root=%lu, diff=%lu ms", 
                 current_time, last_root_announcement, current_time - last_root_announcement);
        heard_from_root = false;
    }
    
    // If election timer expired and no root heard, check for single node scenario
    if (current_time >= election_timer && !heard_from_root && !ble_connected) {
        ESP_LOGI(TAG, "🗳️ No root detected for 30+ seconds - checking for other nodes (Node 0x%04X, BLE: %s, Role: %s)", 
                 node_id, ble_connected ? "connected" : "disconnected", getRoleString());
        
        // Send discovery packet to check for other nodes
        sendElectionDiscoveryPacket();
        
        // Brief wait to allow other nodes to respond
        uint32_t discovery_start = esp_timer_get_time() / 1000;
        while ((esp_timer_get_time() / 1000 - discovery_start) < 2000) {
            vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms for 2 seconds
            if (heard_from_root) {
                ESP_LOGI(TAG, "Other nodes detected during discovery - not alone");
                break;
            }
        }
        
        // If still no root heard, likely single node - become autonomous root immediately  
        if (!heard_from_root) {
            ESP_LOGI(TAG, "🗳️ Single node scenario detected - becoming autonomous root");
            transitionToRole(NodeRole::MESH_ROOT_AUTONOMOUS);
            
            // Debug BLE stack status after election
            esp_bt_controller_status_t bt_status = esp_bt_controller_get_status();
            ESP_LOGI(TAG, "🔍 BLE stack status after election: %d (0=idle, 1=inited, 2=enabled)", bt_status);
            ESP_LOGI(TAG, "🔍 Checking BLE advertising status after becoming autonomous root");
            
            // Debug GATT service state after election
            ESP_LOGI(TAG, "🔍 GATT service validation after autonomous election:");
            ESP_LOGI(TAG, "🔍 Testing if GATT callbacks are still registered...");
            
            // Trigger GATT health validation 
            ESP_LOGI(TAG, "🔍 Calling static GATT health check...");
            BLEGattServer::staticValidateGattHealth();
            
            sendRootAnnouncement();
        } else {
            ESP_LOGI(TAG, "Other nodes detected - starting full advanced election");
            startAdvancedElection();
        }
        
        // Reset election timer to prevent immediate re-triggering
        election_timer = current_time + 60000; // Wait 60 seconds before next election attempt
    }
}

void ESPNowMeshCoordinator::becomeAutonomousRoot() {
    ESP_LOGI(TAG, "Becoming autonomous root node (no mobile phone required)");
    
    // Send election packet to see if anyone objects
    sendElectionPacket();
    
    // Wait a short time for objections
    vTaskDelay(pdMS_TO_TICKS(1000 + (node_id % 2000))); // 1-3 second wait
    
    // If no BLE root appeared, become autonomous root
    if (!heard_from_root && !ble_connected) {
        transitionToRole(NodeRole::MESH_ROOT_AUTONOMOUS);
        sendRootAnnouncement();
    }
}

void ESPNowMeshCoordinator::sendRootAnnouncement() {
    const char* root_type = (current_role == NodeRole::MESH_ROOT_ACTIVE) ? "BLE root" : "autonomous root";
    uint32_t ble_age = getBleConnectionAge();
    
    ESP_LOGI(TAG, "Announcing %s status to mesh (BLE age: %lu ms)", root_type, ble_age);
    
    // Create announcement packet with root type and BLE priority information
    GenericPacket announcement;
    uint8_t announcement_data[12];  // Extended from 8 to 12 bytes
    announcement_data[0] = 0xAA; // Root announcement magic byte 1
    announcement_data[1] = 0xBB; // Root announcement magic byte 2  
    announcement_data[2] = 0xCC; // Root announcement magic byte 3
    announcement_data[3] = 0xDD; // Root announcement magic byte 4
    announcement_data[4] = (node_id >> 8) & 0xFF; // Node ID high byte
    announcement_data[5] = node_id & 0xFF;        // Node ID low byte
    announcement_data[6] = ble_connected ? 1 : 0; // Root type: 1 = BLE root (highest priority), 0 = autonomous root
    announcement_data[7] = (current_role == NodeRole::MESH_ROOT_ACTIVE) ? 1 : 0; // Confirmation of BLE status
    
    // BLE connection age (32-bit) - lower value = more recent connection = higher priority
    announcement_data[8] = (ble_age >> 24) & 0xFF;  // BLE age byte 3 (MSB)
    announcement_data[9] = (ble_age >> 16) & 0xFF;  // BLE age byte 2
    announcement_data[10] = (ble_age >> 8) & 0xFF;  // BLE age byte 1  
    announcement_data[11] = ble_age & 0xFF;         // BLE age byte 0 (LSB)
    
    announcement.setData(announcement_data, sizeof(announcement_data));
    
    ESPNowMeshPacket packet = createMeshPacket(MeshPacketType::ROOT_ANNOUNCEMENT, announcement, 4);
    sendMeshPacket(packet);
}

esp_err_t ESPNowMeshCoordinator::sendElectionPacket() {
    ESP_LOGI(TAG, "Sending root election packet (Node ID: 0x%04X)", node_id);
    
    // Create election packet with our node priority
    GenericPacket election;
    uint8_t election_data[8];
    election_data[0] = 0x52; // 'R' - Election magic byte
    election_data[1] = 0x4F; // 'O'
    election_data[2] = 0x4F; // 'O'
    election_data[3] = 0x54; // 'T'
    election_data[4] = (node_id >> 8) & 0xFF; // Node ID high byte
    election_data[5] = node_id & 0xFF;        // Node ID low byte
    election_data[6] = ble_connected ? 1 : 0; // BLE status
    election_data[7] = (esp_timer_get_time() / 1000000) & 0xFF; // Uptime
    
    election.setData(election_data, sizeof(election_data));
    
    ESPNowMeshPacket packet = createMeshPacket(MeshPacketType::ROOT_ELECTION, election, 4);
    return sendMeshPacket(packet);
}

void ESPNowMeshCoordinator::stepDownIfNeeded() {
    // This method provides a way to trigger stepDown evaluation externally
    // Currently, stepDown logic is handled in ROOT_ANNOUNCEMENT processing
    // This method is here for completeness and potential future use
    
    if (current_role == NodeRole::MESH_ROOT_AUTONOMOUS) {
        ESP_LOGD(TAG, "stepDownIfNeeded called - autonomous root checking for superior roots");
        // In the current implementation, stepDown happens automatically when 
        // BLE root announcements are received in handleReceivedPacket()
    }
}

uint32_t ESPNowMeshCoordinator::getBleConnectionAge() const {
    if (!has_ble_connection_timestamp) {
        return UINT32_MAX;  // No BLE connection = oldest possible (lowest priority)
    }
    
    uint32_t current_uptime = esp_timer_get_time() / 1000;
    
    // Handle uptime rollover (every ~49 days)
    if (current_uptime < ble_connection_uptime_ms) {
        // Rollover occurred, calculate across rollover boundary
        uint32_t time_before_rollover = UINT32_MAX - ble_connection_uptime_ms;
        return time_before_rollover + current_uptime;
    }
    
    // Normal case: current uptime > connection uptime
    return current_uptime - ble_connection_uptime_ms;
}

bool ESPNowMeshCoordinator::shouldAcceptBleConnection() const {
    // Always accept connections if we're not autonomous root
    if (current_role != NodeRole::MESH_ROOT_AUTONOMOUS) {
        return true;
    }
    
    // Check if stabilization period has passed (3 seconds)
    uint32_t current_time = esp_timer_get_time() / 1000;
    uint32_t time_since_autonomous = current_time - autonomous_root_timestamp;
    
    static const uint32_t STABILIZATION_PERIOD_MS = 3000; // 3 seconds
    
    if (time_since_autonomous < STABILIZATION_PERIOD_MS) {
        ESP_LOGW(TAG, "🔒 BLE connection blocked - stabilization period active (%lu/%lu ms)", 
                 time_since_autonomous, STABILIZATION_PERIOD_MS);
        return false;
    }
    
    ESP_LOGI(TAG, "✅ BLE connection allowed - stabilization period complete (%lu ms)", time_since_autonomous);
    return true;
}

uint32_t ESPNowMeshCoordinator::calculateNodePriority() const {
    uint32_t priority = 0;
    uint32_t current_uptime = esp_timer_get_time() / 1000;
    
    // Priority factors (higher score = higher priority):
    
    // 1. BLE connection status (highest priority - 10000 points)
    if (ble_connected) {
        priority += 10000;
    }
    
    // 2. Uptime stability factor (0-1000 points, max at 10+ minutes)
    // Longer uptime = more stable = higher priority
    uint32_t uptime_minutes = current_uptime / 60000;
    uint32_t uptime_score = std::min(uptime_minutes * 100, (uint32_t)1000);  // Cap at 1000 points
    priority += uptime_score;
    
    // 3. MAC address uniqueness factor (0-999 points)  
    // Use last 2 bytes of MAC to ensure deterministic but distributed priority
    uint16_t mac_factor = ((uint16_t)local_mac[4] << 8) | local_mac[5];
    priority += mac_factor % 1000;  // 0-999 points based on MAC
    
    // 4. Hardware capabilities bonus (future expansion)
    // Could add LED count, memory, etc. For now, just a small random factor
    priority += (node_id % 100);  // 0-99 points based on node ID
    
    ESP_LOGD(TAG, "Calculated node priority: %lu (BLE:%s, uptime:%lus, mac_factor:%u)", 
             priority, ble_connected ? "YES" : "NO", current_uptime/1000, mac_factor % 1000);
    
    return priority;
}

void ESPNowMeshCoordinator::startAdvancedElection() {
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Don't start election if we already have a BLE connection (automatic root)
    if (ble_connected) {
        ESP_LOGI(TAG, "Skipping election - BLE connection gives automatic root status");
        return;
    }
    
    // Don't start election too frequently (minimum 30 seconds between attempts)
    if (current_time - last_election_attempt < 30000) {
        ESP_LOGD(TAG, "Election attempt too soon - last attempt %lu ms ago, waiting...", 
                 current_time - last_election_attempt);
        return;
    }
    
    // Don't start election if we're not a client
    if (current_role != NodeRole::MESH_CLIENT) {
        ESP_LOGD(TAG, "Not starting election - current role is not client");
        return;
    }
    
    // Don't start election if one is already in progress
    if (election_state != ElectionState::ELECTION_IDLE) {
        ESP_LOGD(TAG, "Election already in progress (state: %d) - skipping", (int)election_state);
        return;
    }
    
    ESP_LOGI(TAG, "🗳️ Starting advanced autonomous root election");
    ESP_LOGI(TAG, "Node 0x%04X priority: %lu", node_id, calculateNodePriority());
    
    // Initialize election state
    election_state = ElectionState::ELECTION_DISCOVERY;
    election_start_time = current_time;
    last_election_attempt = current_time;
    elected_root_node_id = 0;
    election_candidate_count = 0;
    
    // Add ourselves as a candidate
    ElectionCandidate self_candidate;
    self_candidate.node_id = node_id;
    self_candidate.priority_score = calculateNodePriority();
    self_candidate.uptime_ms = current_time;
    memcpy(self_candidate.mac_addr, local_mac, 6);
    self_candidate.timestamp_received = current_time;
    if (election_candidate_count < MAX_ELECTION_CANDIDATES) {
        election_candidates[election_candidate_count++] = self_candidate;
    }
    
    // Set discovery phase timeout (3-7 seconds randomized to prevent collisions)
    uint32_t discovery_timeout = 3000 + (esp_random() % 4000);  // 3-7 seconds
    election_phase_timeout = current_time + discovery_timeout;
    
    ESP_LOGI(TAG, "Election DISCOVERY phase: listening for candidates (%lu ms timeout)", discovery_timeout);
    ESP_LOGI(TAG, "🗳️ Election initialized: state=DISCOVERY, timeout=%lu, candidates=%zu", 
             election_phase_timeout, election_candidate_count);
    
    // Send discovery announcement to let other nodes know we're starting election
    sendElectionDiscoveryPacket();
}

void ESPNowMeshCoordinator::processElectionPacket(const ESPNowMeshPacket& packet) {
    MeshPacketType packet_type = (MeshPacketType)packet.packet_type;
    
    switch (packet_type) {
        case MeshPacketType::ELECTION_DISCOVERY:
            processElectionDiscovery(packet);
            break;
        case MeshPacketType::ELECTION_CANDIDATE:
            processElectionCandidate(packet);
            break;
        case MeshPacketType::ELECTION_VOTE:
            processElectionVote(packet);
            break;
        case MeshPacketType::ELECTION_RESULT:
            processElectionResult(packet);
            break;
        default:
            ESP_LOGW(TAG, "Unknown election packet type: %d", packet.packet_type);
            break;
    }
}

void ESPNowMeshCoordinator::checkElectionTimeout() {
    if (election_state == ElectionState::ELECTION_IDLE) {
        return;  // No election in progress
    }
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Optional debug logging during election (only if needed for troubleshooting)
    static uint32_t last_election_debug = 0;
    if (current_time - last_election_debug > 5000) {  // Log every 5 seconds during election (reduced frequency)
        ESP_LOGD(TAG, "Election in progress: state=%d, timeout in %ld ms, candidates=%zu", 
                 (int)election_state, 
                 election_phase_timeout > current_time ? election_phase_timeout - current_time : 0,
                 election_candidate_count);
        last_election_debug = current_time;
    }
    
    // Check if current phase has timed out
    if (current_time >= election_phase_timeout) {
        ESP_LOGI(TAG, "Election phase timeout - advancing to next phase (current: %d)", (int)election_state);
        advanceElectionPhase();
    }
}

void ESPNowMeshCoordinator::advanceElectionPhase() {
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    switch (election_state) {
        case ElectionState::ELECTION_DISCOVERY:
            // Discovery phase complete - move to candidate announcement
            ESP_LOGI(TAG, "Election phase: DISCOVERY → CANDIDATE");
            election_state = ElectionState::ELECTION_CANDIDATE;
            election_phase_timeout = current_time + 3000;  // 3 second candidate phase
            sendElectionCandidatePacket();
            break;
            
        case ElectionState::ELECTION_CANDIDATE:
            // Candidate phase complete - move to voting
            ESP_LOGI(TAG, "Election phase: CANDIDATE → VOTING");
            election_state = ElectionState::ELECTION_VOTING;
            election_phase_timeout = current_time + 2000;  // 2 second voting phase
            sendElectionVotePacket();
            break;
            
        case ElectionState::ELECTION_VOTING:
            // Voting phase complete - determine winner and confirm
            ESP_LOGI(TAG, "Election phase: VOTING → CONFIRMED");
            election_state = ElectionState::ELECTION_CONFIRMED;
            election_phase_timeout = current_time + 1000;  // 1 second confirmation phase
            
            // Determine election winner
            if (election_candidate_count > 0) {
                ElectionCandidate winner = selectBestCandidate();
                elected_root_node_id = winner.node_id;
                
                ESP_LOGI(TAG, "🏆 Election winner: Node 0x%04X (priority: %lu)", 
                         winner.node_id, winner.priority_score);
                
                // If we won, become autonomous root
                if (winner.node_id == node_id) {
                    ESP_LOGI(TAG, "🎉 WE WON THE ELECTION! Becoming autonomous root");
                    transitionToRole(NodeRole::MESH_ROOT_AUTONOMOUS);
                } else {
                    ESP_LOGI(TAG, "Election complete - remaining as client");
                    // We stay as client
                }
                
                sendElectionResultPacket();
            } else {
                ESP_LOGW(TAG, "No election candidates found - election failed");
                election_state = ElectionState::ELECTION_IDLE;
            }
            break;
            
        case ElectionState::ELECTION_CONFIRMED:
            // Election complete
            ESP_LOGI(TAG, "Election COMPLETE - returning to idle state");
            election_state = ElectionState::ELECTION_IDLE;
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown election state during timeout: %d", (int)election_state);
            election_state = ElectionState::ELECTION_IDLE;
            break;
    }
}

// Election packet implementations
void ESPNowMeshCoordinator::sendElectionDiscoveryPacket() {
    ESP_LOGI(TAG, "📡 Sending election discovery packet (announcing election start)");
    
    // Create discovery packet with election start announcement
    GenericPacket discovery;
    uint8_t discovery_data[12];
    discovery_data[0] = 0xE1; // Election discovery magic byte 1
    discovery_data[1] = 0xEC; // Election discovery magic byte 2  
    discovery_data[2] = 0x01; // Packet type: Discovery
    discovery_data[3] = 0x00; // Reserved
    discovery_data[4] = (node_id >> 8) & 0xFF; // Node ID high byte
    discovery_data[5] = node_id & 0xFF;        // Node ID low byte
    
    // Election start timestamp for synchronization
    uint32_t election_time = election_start_time;
    discovery_data[6] = (election_time >> 24) & 0xFF;
    discovery_data[7] = (election_time >> 16) & 0xFF;
    discovery_data[8] = (election_time >> 8) & 0xFF;
    discovery_data[9] = election_time & 0xFF;
    
    // Discovery phase timeout
    uint32_t timeout_duration = election_phase_timeout - election_start_time;
    discovery_data[10] = (timeout_duration >> 8) & 0xFF;
    discovery_data[11] = timeout_duration & 0xFF;
    
    discovery.setData(discovery_data, sizeof(discovery_data));
    
    ESPNowMeshPacket packet = createMeshPacket(MeshPacketType::ELECTION_DISCOVERY, discovery, 4);
    sendMeshPacket(packet);
}

void ESPNowMeshCoordinator::sendElectionCandidatePacket() {
    uint32_t our_priority = calculateNodePriority();
    uint32_t current_uptime = esp_timer_get_time() / 1000;
    
    ESP_LOGI(TAG, "🗳️ Sending election candidate packet (priority: %lu, uptime: %lu ms)", our_priority, current_uptime);
    
    // Create candidate packet with our priority information
    GenericPacket candidate;
    uint8_t candidate_data[20];
    candidate_data[0] = 0xE1; // Election magic byte 1
    candidate_data[1] = 0xEC; // Election magic byte 2
    candidate_data[2] = 0x02; // Packet type: Candidate
    candidate_data[3] = 0x00; // Reserved
    
    // Node identification
    candidate_data[4] = (node_id >> 8) & 0xFF; // Node ID high byte
    candidate_data[5] = node_id & 0xFF;        // Node ID low byte
    
    // MAC address for tiebreaker
    memcpy(&candidate_data[6], local_mac, 6);
    
    // Priority score (32-bit)
    candidate_data[12] = (our_priority >> 24) & 0xFF;
    candidate_data[13] = (our_priority >> 16) & 0xFF;
    candidate_data[14] = (our_priority >> 8) & 0xFF;
    candidate_data[15] = our_priority & 0xFF;
    
    // Current uptime (32-bit)
    candidate_data[16] = (current_uptime >> 24) & 0xFF;
    candidate_data[17] = (current_uptime >> 16) & 0xFF;
    candidate_data[18] = (current_uptime >> 8) & 0xFF;
    candidate_data[19] = current_uptime & 0xFF;
    
    candidate.setData(candidate_data, sizeof(candidate_data));
    
    ESPNowMeshPacket packet = createMeshPacket(MeshPacketType::ELECTION_CANDIDATE, candidate, 4);
    sendMeshPacket(packet);
}

void ESPNowMeshCoordinator::sendElectionVotePacket() {
    // Select the best candidate from our list
    if (election_candidate_count == 0) {
        ESP_LOGW(TAG, "No candidates to vote for - skipping vote");
        return;
    }
    
    ElectionCandidate best_candidate = selectBestCandidate();
    ESP_LOGI(TAG, "🗳️ Sending vote for Node 0x%04X (priority: %lu)", best_candidate.node_id, best_candidate.priority_score);
    
    // Create vote packet
    GenericPacket vote;
    uint8_t vote_data[16];
    vote_data[0] = 0xE1; // Election magic byte 1
    vote_data[1] = 0xEC; // Election magic byte 2
    vote_data[2] = 0x03; // Packet type: Vote
    vote_data[3] = 0x00; // Reserved
    
    // Voting node identification
    vote_data[4] = (node_id >> 8) & 0xFF; // Our node ID high byte
    vote_data[5] = node_id & 0xFF;        // Our node ID low byte
    
    // Candidate we're voting for
    vote_data[6] = (best_candidate.node_id >> 8) & 0xFF; // Voted node ID high byte
    vote_data[7] = best_candidate.node_id & 0xFF;        // Voted node ID low byte
    
    // Vote reasoning - candidate's priority score
    vote_data[8] = (best_candidate.priority_score >> 24) & 0xFF;
    vote_data[9] = (best_candidate.priority_score >> 16) & 0xFF;
    vote_data[10] = (best_candidate.priority_score >> 8) & 0xFF;
    vote_data[11] = best_candidate.priority_score & 0xFF;
    
    // Election timestamp for vote validity
    uint32_t vote_time = esp_timer_get_time() / 1000;
    vote_data[12] = (vote_time >> 24) & 0xFF;
    vote_data[13] = (vote_time >> 16) & 0xFF;
    vote_data[14] = (vote_time >> 8) & 0xFF;
    vote_data[15] = vote_time & 0xFF;
    
    vote.setData(vote_data, sizeof(vote_data));
    
    ESPNowMeshPacket packet = createMeshPacket(MeshPacketType::ELECTION_VOTE, vote, 4);
    sendMeshPacket(packet);
}

void ESPNowMeshCoordinator::sendElectionResultPacket() {
    ESP_LOGI(TAG, "🏆 Sending election result packet (winner: Node 0x%04X)", elected_root_node_id);
    
    // Create result packet announcing the election winner
    GenericPacket result;
    uint8_t result_data[16];
    result_data[0] = 0xE1; // Election magic byte 1
    result_data[1] = 0xEC; // Election magic byte 2
    result_data[2] = 0x04; // Packet type: Result
    result_data[3] = 0x00; // Reserved
    
    // Announcing node identification
    result_data[4] = (node_id >> 8) & 0xFF; // Our node ID high byte
    result_data[5] = node_id & 0xFF;        // Our node ID low byte
    
    // Election winner
    result_data[6] = (elected_root_node_id >> 8) & 0xFF; // Winner node ID high byte
    result_data[7] = elected_root_node_id & 0xFF;        // Winner node ID low byte
    
    // Election completion timestamp
    uint32_t completion_time = esp_timer_get_time() / 1000;
    result_data[8] = (completion_time >> 24) & 0xFF;
    result_data[9] = (completion_time >> 16) & 0xFF;
    result_data[10] = (completion_time >> 8) & 0xFF;
    result_data[11] = completion_time & 0xFF;
    
    // Total election duration
    uint32_t election_duration = completion_time - election_start_time;
    result_data[12] = (election_duration >> 24) & 0xFF;
    result_data[13] = (election_duration >> 16) & 0xFF;
    result_data[14] = (election_duration >> 8) & 0xFF;
    result_data[15] = election_duration & 0xFF;
    
    result.setData(result_data, sizeof(result_data));
    
    ESPNowMeshPacket packet = createMeshPacket(MeshPacketType::ELECTION_RESULT, result, 4);
    sendMeshPacket(packet);
}

void ESPNowMeshCoordinator::processElectionDiscovery(const ESPNowMeshPacket& packet) {
    if (packet.data_len < 12) {
        ESP_LOGW(TAG, "Invalid discovery packet size: %d < 12", packet.data_len);
        return;
    }
    
    // Extract source node information
    uint16_t source_node_id = (packet.source_mac[4] << 8) | packet.source_mac[5];
    uint16_t announced_node_id = (packet.data[4] << 8) | packet.data[5];
    
    ESP_LOGI(TAG, "📡 Processing election discovery from Node 0x%04X (announced: 0x%04X)", source_node_id, announced_node_id);
    
    // Validate packet format
    if (packet.data[0] != 0xE1 || packet.data[1] != 0xEC || packet.data[2] != 0x01) {
        ESP_LOGW(TAG, "Invalid discovery packet format");
        return;
    }
    
    // If we're not in an election and someone else is starting one, consider joining
    if (election_state == ElectionState::ELECTION_IDLE && current_role == NodeRole::MESH_CLIENT) {
        // Always join elections to ensure single root selection
        ESP_LOGI(TAG, "🗳️ Joining election started by Node 0x%04X - synchronizing election state", source_node_id);
        
        // JOIN the existing election (don't start our own!)
        uint32_t current_time = esp_timer_get_time() / 1000;
        election_state = ElectionState::ELECTION_DISCOVERY;
        election_start_time = current_time;
        last_election_attempt = current_time;
        elected_root_node_id = 0;
        election_candidate_count = 0;
        
        // Add ourselves as a candidate
        ElectionCandidate self_candidate;
        self_candidate.node_id = node_id;
        self_candidate.priority_score = calculateNodePriority();
        self_candidate.uptime_ms = current_time;
        memcpy(self_candidate.mac_addr, local_mac, 6);
        self_candidate.timestamp_received = current_time;
        if (election_candidate_count < MAX_ELECTION_CANDIDATES) {
            election_candidates[election_candidate_count++] = self_candidate;
        }
        
        // Add the election starter as a candidate
        ElectionCandidate starter_candidate;
        starter_candidate.node_id = source_node_id;
        starter_candidate.priority_score = 0; // Will be updated when they send candidate packet
        starter_candidate.uptime_ms = current_time;
        memcpy(starter_candidate.mac_addr, packet.source_mac, 6);
        starter_candidate.timestamp_received = current_time;
        if (election_candidate_count < MAX_ELECTION_CANDIDATES) {
            election_candidates[election_candidate_count++] = starter_candidate;
        }
        
        // Set discovery phase timeout (shorter since election already started)
        uint32_t remaining_discovery_time = 2000 + (esp_random() % 2000); // 2-4 seconds remaining
        election_phase_timeout = current_time + remaining_discovery_time;
        
        ESP_LOGI(TAG, "🗳️ Joined election: state=DISCOVERY, timeout=%lu, candidates=%zu", 
                 election_phase_timeout, election_candidate_count);
    } else if (election_state != ElectionState::ELECTION_IDLE) {
        ESP_LOGI(TAG, "Already in election - acknowledging Node 0x%04X", source_node_id);
    }
}

void ESPNowMeshCoordinator::processElectionCandidate(const ESPNowMeshPacket& packet) {
    if (packet.data_len < 20) {
        ESP_LOGW(TAG, "Invalid candidate packet size: %d < 20", packet.data_len);
        return;
    }
    
    // Validate packet format
    if (packet.data[0] != 0xE1 || packet.data[1] != 0xEC || packet.data[2] != 0x02) {
        ESP_LOGW(TAG, "Invalid candidate packet format");
        return;
    }
    
    // Extract candidate information
    uint16_t candidate_node_id = (packet.data[4] << 8) | packet.data[5];
    uint32_t priority_score = ((uint32_t)packet.data[12] << 24) |
                              ((uint32_t)packet.data[13] << 16) |
                              ((uint32_t)packet.data[14] << 8) |
                              ((uint32_t)packet.data[15]);
    uint32_t uptime_ms = ((uint32_t)packet.data[16] << 24) |
                         ((uint32_t)packet.data[17] << 16) |
                         ((uint32_t)packet.data[18] << 8) |
                         ((uint32_t)packet.data[19]);
    
    ESP_LOGI(TAG, "🗳️ Processing candidate Node 0x%04X (priority: %lu, uptime: %lu ms)", 
             candidate_node_id, priority_score, uptime_ms);
    
    // Only process candidates if we're in the right election phase
    if (election_state != ElectionState::ELECTION_CANDIDATE && election_state != ElectionState::ELECTION_VOTING) {
        ESP_LOGD(TAG, "Ignoring candidate - not in candidate/voting phase");
        return;
    }
    
    // Check if we already have this candidate
    for (size_t i = 0; i < election_candidate_count; ++i) {
        if (election_candidates[i].node_id == candidate_node_id) {
            // Update existing candidate with newer information
            election_candidates[i].priority_score = priority_score;
            election_candidates[i].uptime_ms = uptime_ms;
            election_candidates[i].timestamp_received = esp_timer_get_time() / 1000;
            memcpy(election_candidates[i].mac_addr, &packet.data[6], 6);
            ESP_LOGD(TAG, "Updated existing candidate Node 0x%04X", candidate_node_id);
            return;
        }
    }
    
    // Add new candidate to our array if space available
    if (election_candidate_count < MAX_ELECTION_CANDIDATES) {
        ElectionCandidate& new_candidate = election_candidates[election_candidate_count];
        new_candidate.node_id = candidate_node_id;
        new_candidate.priority_score = priority_score;
        new_candidate.uptime_ms = uptime_ms;
        new_candidate.timestamp_received = esp_timer_get_time() / 1000;
        memcpy(new_candidate.mac_addr, &packet.data[6], 6);
        
        election_candidate_count++;
        ESP_LOGI(TAG, "Added new candidate Node 0x%04X (total candidates: %zu)", 
                 candidate_node_id, election_candidate_count);
    } else {
        ESP_LOGW(TAG, "Election candidate list full - cannot add Node 0x%04X", candidate_node_id);
    }
}

void ESPNowMeshCoordinator::processElectionVote(const ESPNowMeshPacket& packet) {
    if (packet.data_len < 16) {
        ESP_LOGW(TAG, "Invalid vote packet size: %d < 16", packet.data_len);
        return;
    }
    
    // Validate packet format
    if (packet.data[0] != 0xE1 || packet.data[1] != 0xEC || packet.data[2] != 0x03) {
        ESP_LOGW(TAG, "Invalid vote packet format");
        return;
    }
    
    // Extract vote information
    uint16_t voting_node_id = (packet.data[4] << 8) | packet.data[5];
    uint16_t voted_for_node_id = (packet.data[6] << 8) | packet.data[7];
    uint32_t candidate_priority = ((uint32_t)packet.data[8] << 24) |
                                  ((uint32_t)packet.data[9] << 16) |
                                  ((uint32_t)packet.data[10] << 8) |
                                  ((uint32_t)packet.data[11]);
    
    ESP_LOGI(TAG, "🗳️ Processing vote from Node 0x%04X for Node 0x%04X (priority: %lu)", 
             voting_node_id, voted_for_node_id, candidate_priority);
    
    // Only process votes if we're in voting phase
    if (election_state != ElectionState::ELECTION_VOTING) {
        ESP_LOGD(TAG, "Ignoring vote - not in voting phase");
        return;
    }
    
    // Note: In this implementation, votes are primarily informational
    // The actual winner is determined by objective priority comparison
    // Votes can be used for validation and consensus verification
    
    // Log the vote for debugging and potential future consensus features
    ESP_LOGI(TAG, "Vote recorded: Node 0x%04X → Node 0x%04X", voting_node_id, voted_for_node_id);
    
    // Future enhancement: Could track vote counts for consensus validation
    // For now, we rely on deterministic priority-based selection
}

void ESPNowMeshCoordinator::processElectionResult(const ESPNowMeshPacket& packet) {
    if (packet.data_len < 16) {
        ESP_LOGW(TAG, "Invalid result packet size: %d < 16", packet.data_len);
        return;
    }
    
    // Validate packet format
    if (packet.data[0] != 0xE1 || packet.data[1] != 0xEC || packet.data[2] != 0x04) {
        ESP_LOGW(TAG, "Invalid result packet format");
        return;
    }
    
    // Extract result information
    uint16_t announcing_node_id = (packet.data[4] << 8) | packet.data[5];
    uint16_t winner_node_id = (packet.data[6] << 8) | packet.data[7];
    uint32_t election_duration = ((uint32_t)packet.data[12] << 24) |
                                 ((uint32_t)packet.data[13] << 16) |
                                 ((uint32_t)packet.data[14] << 8) |
                                 ((uint32_t)packet.data[15]);
    
    ESP_LOGI(TAG, "🏆 Processing election result from Node 0x%04X: Winner is Node 0x%04X (duration: %lu ms)", 
             announcing_node_id, winner_node_id, election_duration);
    
    // Accept the election result if we were participating
    if (election_state != ElectionState::ELECTION_IDLE) {
        ESP_LOGI(TAG, "Accepting election result: Node 0x%04X is the new autonomous root", winner_node_id);
        
        // If we won but didn't know it, become autonomous root
        if (winner_node_id == node_id && current_role == NodeRole::MESH_CLIENT) {
            ESP_LOGI(TAG, "🎉 Election result confirms we are the winner - becoming autonomous root");
            transitionToRole(NodeRole::MESH_ROOT_AUTONOMOUS);
        }
        // If someone else won, make sure we're a client (regardless of current role)
        else if (winner_node_id != node_id) {
            if (current_role != NodeRole::MESH_CLIENT) {
                ESP_LOGI(TAG, "Election result shows Node 0x%04X won - stepping down to client (was %s)", 
                         winner_node_id, getRoleString());
                transitionToRole(NodeRole::MESH_CLIENT);
            } else {
                ESP_LOGI(TAG, "Election result shows Node 0x%04X won - we remain client", winner_node_id);
            }
        }
        
        // End our election participation
        election_state = ElectionState::ELECTION_IDLE;
        elected_root_node_id = winner_node_id;
        
        // Reset election timer to prevent immediate new election
        election_timer = esp_timer_get_time() / 1000 + 60000; // Wait 60 seconds
        
        ESP_LOGI(TAG, "Election participation ended - new root: Node 0x%04X", winner_node_id);
    } else {
        ESP_LOGI(TAG, "Election result received but we weren't participating - checking for conflict resolution");
        
        // CRITICAL: Even if we weren't participating, we need to resolve split-brain scenarios
        // If we're autonomous root and another node won an election, we should step down
        if (current_role == NodeRole::MESH_ROOT_AUTONOMOUS && winner_node_id != node_id) {
            ESP_LOGI(TAG, "🚨 Split-brain detected: We are autonomous root but Node 0x%04X won election", winner_node_id);
            ESP_LOGI(TAG, "🚨 Stepping down from autonomous root to prevent split-brain scenario");
            transitionToRole(NodeRole::MESH_CLIENT);
            
            // Reset election timer to prevent immediate re-election
            election_timer = esp_timer_get_time() / 1000 + 60000; // Wait 60 seconds
        }
    }
}

ESPNowMeshCoordinator::ElectionCandidate ESPNowMeshCoordinator::selectBestCandidate() const {
    if (election_candidate_count == 0) {
        // Return a default candidate if none exist
        ElectionCandidate default_candidate = {};
        return default_candidate;
    }
    
    // Find candidate with highest priority (using the < operator defined in struct)
    ElectionCandidate best_candidate = election_candidates[0];
    for (size_t i = 1; i < election_candidate_count; ++i) {
        if (election_candidates[i] < best_candidate) {  // < operator means higher priority
            best_candidate = election_candidates[i];
        }
    }
    
    return best_candidate;
}

// Adaptive Mesh Control Methods
esp_err_t ESPNowMeshCoordinator::enableAdaptiveMesh() {
    if (adaptive_mesh_enabled) {
        ESP_LOGW(TAG, "Adaptive mesh already enabled");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Enabling adaptive mesh system");
    
    // Initialize adaptive mesh components
    if (!neighbor_manager) {
        neighbor_manager = std::make_unique<NeighborManager>();
        if (neighbor_manager->init(node_id) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize NeighborManager");
            neighbor_manager.reset();
            return ESP_FAIL;
        }
    }
    
    if (!topology_manager) {
        topology_manager = std::make_unique<TopologyManager>();
        if (topology_manager->init(node_id, neighbor_manager.get()) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize TopologyManager");
            topology_manager.reset();
            neighbor_manager.reset();
            return ESP_FAIL;
        }
    }
    
    if (!adaptive_router) {
        adaptive_router = std::make_unique<AdaptiveRouter>();
        if (adaptive_router->init(node_id, neighbor_manager.get(), topology_manager.get()) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize AdaptiveRouter");
            adaptive_router.reset();
            topology_manager.reset();
            neighbor_manager.reset();
            return ESP_FAIL;
        }
    }
    
    // Connect beacon send callback to ESP-NOW transmission
    neighbor_manager->setBeaconSendCallback([this](const uint8_t* data, size_t len) {
        // Create ADAPTIVE_NEIGHBOR_DISCOVERY packet from beacon data
        GenericPacket beacon_packet;
        beacon_packet.setData(data, len);
        
        ESPNowMeshPacket mesh_packet = createMeshPacket(
            MeshPacketType::ADAPTIVE_NEIGHBOR_DISCOVERY, 
            beacon_packet, 
            2  // Limited TTL for neighbor discovery
        );
        
        esp_err_t result = sendMeshPacket(mesh_packet);
        if (result == ESP_OK) {
            ESP_LOGD(TAG, "📡 Neighbor discovery beacon transmitted successfully");
        } else {
            ESP_LOGW(TAG, "❌ Failed to transmit neighbor discovery beacon: %s", 
                     esp_err_to_name(result));
        }
    });
    
    // Start adaptive mesh components
    neighbor_manager->startNeighborDiscovery();
    ESP_LOGI(TAG, "Started neighbor discovery process with ESP-NOW transmission");
    
    adaptive_mesh_enabled = true;
    ESP_LOGI(TAG, "Adaptive mesh system enabled successfully");
    
    return ESP_OK;
}

void ESPNowMeshCoordinator::disableAdaptiveMesh() {
    if (!adaptive_mesh_enabled) {
        ESP_LOGW(TAG, "Adaptive mesh already disabled");
        return;
    }
    
    ESP_LOGI(TAG, "Disabling adaptive mesh system");
    
    // Clean shutdown of components
    adaptive_router.reset();
    topology_manager.reset();
    neighbor_manager.reset();
    
    adaptive_mesh_enabled = false;
    ESP_LOGI(TAG, "Adaptive mesh system disabled");
}

bool ESPNowMeshCoordinator::isAdaptiveMeshEnabled() const {
    return adaptive_mesh_enabled;
}

size_t ESPNowMeshCoordinator::getActiveNeighborCount() const {
    if (neighbor_manager) {
        return neighbor_manager->getActiveNeighborCount();
    }
    return 0;
}

void ESPNowMeshCoordinator::printAdaptiveMeshStatus() const {
    if (!adaptive_mesh_enabled) {
        ESP_LOGI(TAG, "🔄 Adaptive mesh system is DISABLED - using flat mesh");
        return;
    }
    
    ESP_LOGI(TAG, "🔍 === ADAPTIVE MESH STATUS ===");
    
    // NeighborManager status
    if (neighbor_manager) {
        const auto& neighbor_stats = neighbor_manager->getStats();
        ESP_LOGI(TAG, "📡 Neighbor Discovery:");
        ESP_LOGI(TAG, "  • Active Neighbors: %d", neighbor_stats.current_neighbor_count);
        ESP_LOGI(TAG, "  • Total Discovered: %lu", neighbor_stats.neighbors_discovered);
        ESP_LOGI(TAG, "  • Beacons Sent: %lu", neighbor_stats.beacons_sent);
        ESP_LOGI(TAG, "  • Beacons Received: %lu", neighbor_stats.beacons_received);
        
        if (neighbor_stats.current_neighbor_count > 0) {
            neighbor_manager->printNeighborTable();
        }
    }
    
    // TopologyManager status  
    if (topology_manager) {
        const auto& topo_stats = topology_manager->getStats();
        ESP_LOGI(TAG, "🗺️ Network Topology:");
        ESP_LOGI(TAG, "  • Reachable Nodes: %zu", topology_manager->getReachableNodeCount());
        ESP_LOGI(TAG, "  • Topology Updates Sent: %lu", topo_stats.topology_updates_sent);
        ESP_LOGI(TAG, "  • Topology Updates Received: %lu", topo_stats.topology_updates_received);
        ESP_LOGI(TAG, "  • Role Changes: %lu", topo_stats.role_changes);
    }
    
    // AdaptiveRouter status
    if (adaptive_router) {
        const auto& routing_stats = adaptive_router->getStats();
        ESP_LOGI(TAG, "🛣️ Adaptive Routing:");
        ESP_LOGI(TAG, "  • Active Destinations: %u", routing_stats.active_destinations);
        ESP_LOGI(TAG, "  • Total Routes: %u", routing_stats.total_routes);
        ESP_LOGI(TAG, "  • Route Updates Sent: %lu", routing_stats.route_updates_sent);
        ESP_LOGI(TAG, "  • Route Updates Received: %lu", routing_stats.route_updates_received);
        ESP_LOGI(TAG, "  • Packets Routed: %lu", routing_stats.packets_routed);
        ESP_LOGI(TAG, "  • Route Discoveries: %lu", routing_stats.route_discoveries);
        
        if (routing_stats.total_routes > 0) {
            adaptive_router->printRoutingTable();
        }
    }
    
    ESP_LOGI(TAG, "🔍 === END ADAPTIVE MESH STATUS ===");
}

void ESPNowMeshCoordinator::simulateNeighborDiscovery(uint16_t simulated_node_id, int8_t rssi) {
    if (!adaptive_mesh_enabled || !neighbor_manager) {
        ESP_LOGW(TAG, "Adaptive mesh not enabled - cannot simulate neighbor discovery");
        return;
    }
    
    ESP_LOGI(TAG, "🧪 Simulating neighbor discovery from Node 0x%04X (RSSI: %d dBm)", 
             simulated_node_id, rssi);
    
    // Create a simulated neighbor discovery beacon
    struct SimulatedBeacon {
        uint32_t magic = 0xDEADBEEF;  // Match NeighborManager's DISCOVERY_BEACON_MAGIC
        uint16_t node_id;
        uint8_t beacon_sequence = 1;
        uint32_t uptime_ms;
        uint8_t neighbor_count = 0;
        uint32_t timestamp_ms;
    } __attribute__((packed));
    
    SimulatedBeacon beacon;
    beacon.node_id = simulated_node_id;
    beacon.uptime_ms = esp_timer_get_time() / 1000;
    beacon.timestamp_ms = beacon.uptime_ms;
    
    // Create a fake MAC address for this simulated node
    uint8_t simulated_mac[6] = {0x02, 0x00, 0x00, 0x00, 
                                (uint8_t)(simulated_node_id >> 8), 
                                (uint8_t)(simulated_node_id & 0xFF)};
    
    // Send the beacon to NeighborManager for processing
    neighbor_manager->processNeighborBeacon(simulated_mac, 
                                           (uint8_t*)&beacon, 
                                           sizeof(beacon), 
                                           rssi);
    
    ESP_LOGI(TAG, "🧪 Neighbor discovery simulation completed");
}

// Adaptive Mesh Packet Processing Methods
void ESPNowMeshCoordinator::processAdaptiveNeighborDiscovery(const ESPNowMeshPacket& packet) {
    if (!neighbor_manager) {
        ESP_LOGW(TAG, "NeighborManager not initialized");
        return;
    }
    
    ESP_LOGD(TAG, "🔍 Processing neighbor discovery beacon from Node 0x%04X", 
             (packet.source_mac[4] << 8) | packet.source_mac[5]);
    
    // RSSI would come from ESP-NOW receive callback - using reasonable default
    // In production, this would be passed from the actual radio layer
    int8_t rssi = -60; // Reasonable default for close neighbors
    
    neighbor_manager->processNeighborBeacon(packet.source_mac, packet.data, 
                                          packet.data_len, rssi);
}

void ESPNowMeshCoordinator::processAdaptiveTopologyUpdate(const ESPNowMeshPacket& packet) {
    if (!topology_manager) {
        ESP_LOGW(TAG, "TopologyManager not initialized");
        return;
    }
    
    ESP_LOGD(TAG, "Processing adaptive topology update packet");
    
    // Pass topology update to TopologyManager with sender MAC
    topology_manager->processTopologyUpdate(packet.source_mac, packet.data, packet.data_len);
}

void ESPNowMeshCoordinator::processAdaptiveRouting(const ESPNowMeshPacket& packet) {
    if (!adaptive_router) {
        ESP_LOGW(TAG, "AdaptiveRouter not initialized");
        return;
    }
    
    ESP_LOGD(TAG, "Processing adaptive routing packet (type: %d)", packet.packet_type);
    
    // Pass route updates to AdaptiveRouter with sender MAC
    adaptive_router->processRouteUpdate(packet.source_mac, packet.data, packet.data_len);
}

void ESPNowMeshCoordinator::processAdaptiveDataForward(const ESPNowMeshPacket& packet) {
    if (!adaptive_router) {
        ESP_LOGW(TAG, "AdaptiveRouter not initialized");
        return;
    }
    
    ESP_LOGD(TAG, "Processing adaptive data forward packet");
    
    // For now, this is a placeholder for smart packet forwarding
    // In the future, this would use the adaptive router to determine
    // the best forwarding strategy based on topology and routes
    ESP_LOGD(TAG, "Adaptive data forwarding not yet implemented");
}