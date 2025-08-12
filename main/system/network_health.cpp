#include "network_health.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>

const char* NetworkHealthMonitor::TAG = "NetworkHealth";

NetworkHealthMonitor::NetworkHealthMonitor() 
    : total_packets_sent(0)
    , total_packets_failed(0)
    , start_time_ms(esp_timer_get_time() / 1000)
{
    memset(&current_health, 0, sizeof(current_health));
}

void NetworkHealthMonitor::updateMetrics(const MeshStats& mesh_stats, 
                                       uint8_t neighbor_count, 
                                       int8_t avg_rssi,
                                       uint8_t role,
                                       uint8_t total_nodes) {
    // Update packet statistics
    total_packets_sent = mesh_stats.packets_sent;
    total_packets_failed = mesh_stats.send_failures;
    
    // Update current health metrics
    current_health.active_neighbors = neighbor_count;
    current_health.avg_signal_strength = avg_rssi;
    current_health.mesh_role = role;
    current_health.total_nodes = total_nodes;
    current_health.packet_success_rate = calculateSuccessRate();
    current_health.uptime_hours = (uint16_t)((esp_timer_get_time() / 1000 - start_time_ms) / 3600000);
    current_health.overall_score = calculateOverallScore();
}

NetworkHealth NetworkHealthMonitor::getCurrentHealth() const {
    return current_health;
}

NetworkHealthLevel NetworkHealthMonitor::getHealthLevel() const {
    if (current_health.overall_score >= NETWORK_EXCELLENT) {
        return NETWORK_EXCELLENT;
    } else if (current_health.overall_score >= NETWORK_GOOD) {
        return NETWORK_GOOD;  
    } else if (current_health.overall_score >= NETWORK_POOR) {
        return NETWORK_POOR;
    } else {
        return NETWORK_CRITICAL;
    }
}

void NetworkHealthMonitor::reset() {
    total_packets_sent = 0;
    total_packets_failed = 0;
    start_time_ms = esp_timer_get_time() / 1000;
    memset(&current_health, 0, sizeof(current_health));
}

uint8_t NetworkHealthMonitor::calculateOverallScore() const {
    uint8_t score = 0;
    
    // Packet success rate (35% weight)
    score += (current_health.packet_success_rate * 35) / 100;
    
    // Signal strength (25% weight) - convert RSSI to 0-100 scale
    // RSSI: -30 = excellent, -50 = good, -70 = fair, -90 = poor
    int rssi_score = 100;
    if (current_health.avg_signal_strength < -90) {
        rssi_score = 0;
    } else if (current_health.avg_signal_strength < -70) {
        rssi_score = 25;
    } else if (current_health.avg_signal_strength < -50) {
        rssi_score = 75;
    }
    score += (rssi_score * 25) / 100;
    
    // Neighbor connectivity (30% weight)  
    // 0 neighbors = 0, 1-2 = 50%, 3+ = 100%
    int neighbor_score = 100;
    if (current_health.active_neighbors == 0) {
        neighbor_score = 0;
    } else if (current_health.active_neighbors < 3) {
        neighbor_score = 50;
    }
    score += (neighbor_score * 30) / 100;

    // Total nodes (10% weight)
    // 1-3 nodes = 50%, 4-7 = 75%, 8+ = 100%
    int total_nodes_score = 100;
    if (current_health.total_nodes < 4) {
        total_nodes_score = 50;
    } else if (current_health.total_nodes < 8) {
        total_nodes_score = 75;
    }
    score += (total_nodes_score * 10) / 100;
    
    return (score > 100) ? 100 : score;
}

uint8_t NetworkHealthMonitor::calculateSuccessRate() const {
    if (total_packets_sent == 0) {
        return 100; // No packets sent yet, assume healthy
    }
    
    uint32_t successful = total_packets_sent - total_packets_failed;
    return (uint8_t)((successful * 100) / total_packets_sent);
}