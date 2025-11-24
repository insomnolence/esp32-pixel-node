#include "unity.h"
#include "mesh/espnow_mesh_coordinator.h"
#include "mesh/adaptive/neighbor_manager.h"
#include <array>
#include <cstring>

TEST_CASE("BoundedPacketTracker detects duplicates and evicts old entries", "[mesh][packet_tracker]") {
    BoundedPacketTracker tracker;
    const uint32_t packet_id = 0xABCDEF01;

    TEST_ASSERT_FALSE(tracker.isPacketSeen(packet_id));  // first time -> unseen
    TEST_ASSERT_TRUE(tracker.isPacketSeen(packet_id));   // immediately seen

    // Push many unique IDs to force eviction of the original packet_id
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t id = 0x10000000 + i;
        TEST_ASSERT_FALSE(tracker.isPacketSeen(id));
    }

    // Original ID should have been evicted and appear unknown again
    TEST_ASSERT_FALSE(tracker.isPacketSeen(packet_id));
}

static NeighborDiscoveryBeacon makeBeacon(uint16_t node_id) {
    NeighborDiscoveryBeacon beacon{};
    beacon.magic = 0xDEADBEEF;
    beacon.node_id = node_id;
    beacon.beacon_sequence = 1;
    beacon.uptime_ms = 500;
    beacon.neighbor_count = 0;
    beacon.timestamp_ms = 600;
    return beacon;
}

TEST_CASE("NeighborManager classifies link quality correctly", "[mesh][neighbor]") {
    NeighborManager mgr;
    TEST_ASSERT_EQUAL(LinkQuality::EXCELLENT, mgr.assessLinkQuality(-45));
    TEST_ASSERT_EQUAL(LinkQuality::GOOD, mgr.assessLinkQuality(-60));
    TEST_ASSERT_EQUAL(LinkQuality::FAIR, mgr.assessLinkQuality(-72));
    TEST_ASSERT_EQUAL(LinkQuality::POOR, mgr.assessLinkQuality(-84));
    TEST_ASSERT_EQUAL(LinkQuality::UNUSABLE, mgr.assessLinkQuality(-90));
}

TEST_CASE("NeighborManager processes valid beacons and updates neighbors", "[mesh][neighbor]") {
    NeighborManager mgr;
    TEST_ASSERT_EQUAL(ESP_OK, mgr.init(1));   // local node id = 1
    TEST_ASSERT_EQUAL(ESP_OK, mgr.start());

    bool callback_called = false;
    NeighborInfo callback_info;
    mgr.setNeighborUpdateCallback(
        [&](const NeighborInfo& info, bool is_new) {
            callback_called = true;
            callback_info = info;
            TEST_ASSERT_TRUE(is_new);
        });

    uint8_t mac[6] = {0xAA, 0xBB, 0xCC, 0x00, 0x11, 0x22};
    NeighborDiscoveryBeacon beacon = makeBeacon(2);

    mgr.processNeighborBeacon(mac, reinterpret_cast<uint8_t*>(&beacon), sizeof(beacon), -55);

    TEST_ASSERT_TRUE(callback_called);
    const NeighborInfo* neighbor = mgr.getNeighbor(mac);
    TEST_ASSERT_NOT_NULL(neighbor);
    TEST_ASSERT_EQUAL_UINT16(2, neighbor->node_id);
    TEST_ASSERT_TRUE(neighbor->is_active);
    TEST_ASSERT_EQUAL_INT8(-55, neighbor->current_rssi);
    TEST_ASSERT_TRUE(neighbor->beacon_count >= 1);
    TEST_ASSERT_EQUAL_UINT32(1, mgr.getStats().beacons_received);

    // Second beacon should update RSSI and not be considered new
    callback_called = false;
    mgr.setNeighborUpdateCallback(
        [&](const NeighborInfo& info, bool is_new) {
            callback_called = true;
            TEST_ASSERT_FALSE(is_new);
            TEST_ASSERT_EQUAL_INT8(-65, info.current_rssi);
        });

    mgr.processNeighborBeacon(mac, reinterpret_cast<uint8_t*>(&beacon), sizeof(beacon), -65);
    TEST_ASSERT_TRUE(callback_called);
}

TEST_CASE("NeighborManager ignores invalid beacons", "[mesh][neighbor]") {
    NeighborManager mgr;
    TEST_ASSERT_EQUAL(ESP_OK, mgr.init(1));
    TEST_ASSERT_EQUAL(ESP_OK, mgr.start());

    uint8_t mac[6] = {0x01, 0x02, 0x03, 0x10, 0x11, 0x12};
    NeighborDiscoveryBeacon invalid = makeBeacon(3);
    invalid.magic = 0x12345678; // wrong magic

    mgr.processNeighborBeacon(mac, reinterpret_cast<uint8_t*>(&invalid), sizeof(invalid), -60);
    TEST_ASSERT_NULL(mgr.getNeighbor(mac));
    TEST_ASSERT_EQUAL_UINT32(0, mgr.getStats().beacons_received);
}
