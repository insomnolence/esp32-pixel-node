#include "espnow_mesh_coordinator.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include <string.h>
#include <ctime>

const char* ESPNowMeshCoordinator::TAG = "ESPNowMeshCoordinator";
ESPNowMeshCoordinator* ESPNowMeshCoordinator::instance = nullptr;

// PacketTracker implementation
bool PacketTracker::isPacketSeen(uint32_t packet_id) {
    uint32_t now = esp_timer_get_time() / 1000; // Convert to ms
    
    // Periodic cleanup
    if (now - last_cleanup > CLEANUP_INTERVAL_MS) {
        cleanup();
        last_cleanup = now;
    }
    
    if (seen_packets.count(packet_id)) {
        return true; // Already seen
    }
    
    seen_packets.insert(packet_id);
    return false; // New packet
}

void PacketTracker::cleanup() {
    // Clear old packets to prevent memory growth
    seen_packets.clear();
    ESP_LOGD("PacketTracker", "Packet tracker cleaned up");
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
{
    instance = this;
    memset(local_mac, 0, 6);
}

ESPNowMeshCoordinator::~ESPNowMeshCoordinator() {
    stop();
    instance = nullptr;
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
    
    // Start autonomous root election timer (10-30 seconds random delay)
    uint32_t election_delay = 10000 + (esp_random() % 20000); // 10-30 seconds
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
    ESP_LOGI(TAG, "BLE connected - becoming mesh root with mobile control");
    ble_connected = true;
    
    // BLE connection always takes priority over autonomous root
    if (current_role == NodeRole::MESH_ROOT_AUTONOMOUS) {
        ESP_LOGI(TAG, "Upgrading from autonomous root to BLE-controlled root");
    }
    
    transitionToRole(NodeRole::MESH_ROOT_ACTIVE);
}

void ESPNowMeshCoordinator::onBleDisconnected() {
    ESP_LOGI(TAG, "BLE disconnected");
    ble_connected = false;
    
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
    if (instance) {
        ESP_LOGD(instance->TAG, "ESP-NOW send status: %s", 
                 status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
    }
}

void ESPNowMeshCoordinator::onESPNowReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (instance) {
        instance->handleReceivedPacket(recv_info->src_addr, data, len);
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
            ESP_LOGI(TAG, "Root election packet received from node 0x%04X", 
                     (mesh_packet->source_mac[4] << 8) | mesh_packet->source_mac[5]);
            // Reset our election timer if someone else is trying to become root
            election_timer = esp_timer_get_time() / 1000 + 15000; // Wait 15 more seconds
            break;
            
        case MeshPacketType::ROOT_ANNOUNCEMENT:
            ESP_LOGI(TAG, "Root announcement received from node 0x%04X", 
                     (mesh_packet->source_mac[4] << 8) | mesh_packet->source_mac[5]);
            heard_from_root = true;
            last_root_announcement = esp_timer_get_time() / 1000;
            // Cancel our own election if we were about to become root
            if (current_role == NodeRole::MESH_CLIENT) {
                election_timer = esp_timer_get_time() / 1000 + 45000; // Wait 45 seconds before next attempt
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
    
    // Forward with random backoff
    sendMeshPacket(forward_packet);
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
    
    // Only clients can participate in autonomous root election
    if (current_role != NodeRole::MESH_CLIENT) {
        return;
    }
    
    // Check if we haven't heard from any root recently
    if (current_time - last_root_announcement > 30000) { // 30 seconds without root
        heard_from_root = false;
    }
    
    // If election timer expired and no root heard, try to become autonomous root
    if (current_time >= election_timer && !heard_from_root && !ble_connected) {
        ESP_LOGI(TAG, "No root detected for 30+ seconds - attempting autonomous root election");
        becomeAutonomousRoot();
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
    ESP_LOGI(TAG, "Announcing autonomous root status to mesh");
    
    // Create simple announcement packet
    GenericPacket announcement;
    uint8_t announcement_data[] = {0xAA, 0xBB, 0xCC, 0xDD}; // Magic bytes for root announcement
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