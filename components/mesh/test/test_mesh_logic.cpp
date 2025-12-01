#include "unity.h"
#include "mesh/espnow_mesh_coordinator.h"
#include "mesh/neighbor_tracker.h"
#include <array>
#include <cstring>

TEST_CASE("BoundedPacketTracker detects duplicates and evicts old entries", "[mesh][packet_tracker]") {
    BoundedPacketTracker tracker;
    const uint32_t packet_id = 0xABCDEF01;

    // isNewPacket returns TRUE for new packets (and marks them as seen)
    TEST_ASSERT_TRUE(tracker.isNewPacket(packet_id));   // first time -> new
    TEST_ASSERT_FALSE(tracker.isNewPacket(packet_id));  // immediately duplicate

    // Push many unique IDs to force eviction of the original packet_id
    // (history size is 128, so 256 should evict the original)
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t id = 0x10000000 + i;
        TEST_ASSERT_TRUE(tracker.isNewPacket(id));  // All new packets
    }

    // Original ID should have been evicted and appear as new again
    TEST_ASSERT_TRUE(tracker.isNewPacket(packet_id));
}

TEST_CASE("NeighborTracker tracks neighbors and computes average RSSI", "[mesh][neighbor]") {
    NeighborTracker tracker(1); // local node id = 1

    // Initially no neighbors
    TEST_ASSERT_EQUAL_UINT8(0, tracker.getNeighborCount());
    TEST_ASSERT_EQUAL_INT8(0, tracker.getAverageRssi());

    // Add first neighbor
    uint8_t mac1[6] = {0xAA, 0xBB, 0xCC, 0x00, 0x11, 0x22};
    tracker.onPacketReceived(mac1, -50);
    TEST_ASSERT_EQUAL_UINT8(1, tracker.getNeighborCount());
    TEST_ASSERT_EQUAL_INT8(-50, tracker.getAverageRssi());

    // Add second neighbor with different RSSI
    uint8_t mac2[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    tracker.onPacketReceived(mac2, -70);
    TEST_ASSERT_EQUAL_UINT8(2, tracker.getNeighborCount());
    TEST_ASSERT_EQUAL_INT8(-60, tracker.getAverageRssi()); // Average of -50 and -70

    // Update existing neighbor's RSSI
    tracker.onPacketReceived(mac1, -60);
    TEST_ASSERT_EQUAL_UINT8(2, tracker.getNeighborCount()); // Still 2 neighbors
    TEST_ASSERT_EQUAL_INT8(-65, tracker.getAverageRssi()); // Average of -60 and -70
}
