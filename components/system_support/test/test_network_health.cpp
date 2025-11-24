#include "unity.h"
#include "system/network_health.h"

static MeshStats makeStats(uint32_t sent, uint32_t failures) {
    MeshStats stats{};
    stats.packets_sent = sent;
    stats.send_failures = failures;
    stats.packets_received = sent - failures;
    stats.packets_dropped = failures;
    return stats;
}

TEST_CASE("NetworkHealthMonitor computes success rate and score weighting", "[system_support]") {
    NetworkHealthMonitor monitor;
    MeshStats stats = makeStats(100, 25); // 75% success

    monitor.updateMetrics(stats, /*neighbor_count=*/2, /*avg_rssi=*/-60,
                          /*role=*/0, /*total_nodes=*/3);

    NetworkHealth health = monitor.getCurrentHealth();
    TEST_ASSERT_EQUAL_UINT8(75, health.packet_success_rate);
    TEST_ASSERT_EQUAL_UINT8(64, health.overall_score); // derived from weighting
    TEST_ASSERT_EQUAL(NETWORK_POOR, monitor.getHealthLevel());
}

TEST_CASE("NetworkHealthMonitor health levels follow thresholds", "[system_support]") {
    NetworkHealthMonitor monitor;

    // Excellent network
    monitor.updateMetrics(makeStats(200, 0), /*neighbors=*/5, /*rssi=*/-40,
                          /*role=*/1, /*total_nodes=*/10);
    TEST_ASSERT_EQUAL_UINT8(100, monitor.getCurrentHealth().overall_score);
    TEST_ASSERT_EQUAL(NETWORK_EXCELLENT, monitor.getHealthLevel());

    // Degraded network (critical)
    monitor.updateMetrics(makeStats(20, 15), /*neighbors=*/0, /*rssi=*/-95,
                          /*role=*/0, /*total_nodes=*/1);
    NetworkHealth degraded = monitor.getCurrentHealth();
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(30, degraded.overall_score);
    TEST_ASSERT_EQUAL(NETWORK_CRITICAL, monitor.getHealthLevel());
}

TEST_CASE("NetworkHealthMonitor reset clears counters", "[system_support]") {
    NetworkHealthMonitor monitor;
    monitor.updateMetrics(makeStats(50, 10), 3, -55, 1, 4);

    monitor.reset();
    NetworkHealth cleared = monitor.getCurrentHealth();
    TEST_ASSERT_EQUAL_UINT8(0, cleared.overall_score);
    TEST_ASSERT_EQUAL_UINT8(0, cleared.active_neighbors);
    TEST_ASSERT_EQUAL_UINT8(0, cleared.total_nodes);
    TEST_ASSERT_EQUAL_UINT8(0, cleared.packet_success_rate);

    // Fresh update after reset should still work
    monitor.updateMetrics(makeStats(10, 0), 1, -45, 2, 2);
    TEST_ASSERT_EQUAL_UINT8(100, monitor.getCurrentHealth().packet_success_rate);
}
