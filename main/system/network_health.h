#ifndef NETWORK_HEALTH_H_
#define NETWORK_HEALTH_H_

#include <stdint.h>
#include "esp_err.h"

// Simple mesh statistics structure (mirrors ESPNowMeshCoordinator::NetworkStats)
struct MeshStats {
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_dropped;
    uint32_t send_failures;
};

// Minimal network health metrics for BLE reporting
struct NetworkHealth {
    uint8_t overall_score;        // 0-100 health percentage
    uint8_t active_neighbors;     // Number of active mesh neighbors  
    uint8_t packet_success_rate;  // 0-100 percentage of successful transmissions
    int8_t avg_signal_strength;   // Average RSSI in dBm
    uint16_t uptime_hours;        // System uptime in hours
    uint8_t mesh_role;            // 0=client, 1=root_ble, 2=root_autonomous
    uint8_t total_nodes;          // Total nodes detected in mesh network
} __attribute__((packed));

// Network health status levels
enum NetworkHealthLevel {
    NETWORK_EXCELLENT = 90,  // Green - everything working great
    NETWORK_GOOD = 70,       // Yellow - some degradation  
    NETWORK_POOR = 40,       // Orange - significant issues
    NETWORK_CRITICAL = 0     // Red - major problems
};

// Lightweight network health calculator
class NetworkHealthMonitor {
public:
    NetworkHealthMonitor();
    
    // Update metrics from mesh coordinator
    void updateMetrics(const MeshStats& mesh_stats, 
                      uint8_t neighbor_count, 
                      int8_t avg_rssi,
                      uint8_t role,
                      uint8_t total_nodes);
    
    // Get current health for BLE reporting
    NetworkHealth getCurrentHealth() const;
    
    // Get simple health level for LED indicators
    NetworkHealthLevel getHealthLevel() const;
    
    // Reset statistics (for testing/debugging)
    void reset();

private:
    NetworkHealth current_health;
    uint32_t total_packets_sent;
    uint32_t total_packets_failed;
    uint32_t start_time_ms;
    
    uint8_t calculateOverallScore() const;
    uint8_t calculateSuccessRate() const;
    static const char* TAG;
};

#endif // NETWORK_HEALTH_H_