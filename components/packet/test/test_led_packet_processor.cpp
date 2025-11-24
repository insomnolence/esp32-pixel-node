#include "unity.h"
#include "packet/led_packet_processor.h"
#include "packet/generic_packet.h"
#include "packet/packet.h"
#include <array>
#include <string>

namespace {
struct CallbackSpy {
    bool called = false;
    std::string format;
    size_t last_length = 0;
};
} // namespace

static GenericPacket makeMobilePacket(uint8_t command) {
    std::array<uint8_t, 19> payload{};
    payload[0] = command;
    payload[1] = 200; // brightness
    payload[2] = 10;  // speed
    payload[3] = 5;   // pattern
    // Colors
    uint32_t colors[3] = {0x112233, 0x445566, 0x778899};
    memcpy(&payload[4], colors, sizeof(colors));
    payload[16] = 10;
    payload[17] = 20;
    payload[18] = 30;
    return GenericPacket(payload.data(), payload.size());
}

TEST_CASE("LedPacketProcessor rejects invalid packets", "[packet]") {
    LedPacketProcessor processor;
    CallbackSpy spy;
    processor.setLedControlCallback(
        [&spy](const GenericPacket& pkt, const char* format) {
            spy.called = true;
            spy.last_length = pkt.getLength();
            spy.format = format ? format : "";
        });

    GenericPacket empty_packet;
    TEST_ASSERT_FALSE(empty_packet.isValid());
    TEST_ASSERT_FALSE(processor.processPacket(empty_packet));
    TEST_ASSERT_FALSE(spy.called);
}

TEST_CASE("LedPacketProcessor processes 19-byte mobile packets", "[packet]") {
    LedPacketProcessor processor;
    CallbackSpy spy;
    processor.setLedControlCallback(
        [&spy](const GenericPacket& pkt, const char* format) {
            spy.called = true;
            spy.last_length = pkt.getLength();
            spy.format = format ? format : "";
        });

    GenericPacket packet = makeMobilePacket(0x5A);
    TEST_ASSERT_TRUE(processor.canProcessPacket(packet));
    TEST_ASSERT_TRUE(processor.processPacket(packet));
    TEST_ASSERT_TRUE(spy.called);
    TEST_ASSERT_EQUAL_UINT32(19, spy.last_length);
    TEST_ASSERT_EQUAL_STRING("Current LED Packet V1", spy.format.c_str());
}

TEST_CASE("LedPacketProcessor processes struct-sized packets", "[packet]") {
    LedPacketProcessor processor;
    CallbackSpy spy;
    processor.setLedControlCallback(
        [&spy](const GenericPacket& pkt, const char* format) {
            spy.called = true;
            spy.last_length = pkt.getLength();
            spy.format = format ? format : "";
        });

    Packet pkt = {};
    pkt.command = 2;
    pkt.brightness = 150;
    pkt.speed = 12;
    pkt.pattern = 3;
    pkt.color[0] = 0xABCDEF;
    pkt.level[0] = 45;

    GenericPacket container;
    TEST_ASSERT_TRUE(container.setPacket(pkt));
    TEST_ASSERT_EQUAL_UINT32(sizeof(Packet), container.getLength());

    TEST_ASSERT_TRUE(processor.canProcessPacket(container));
    TEST_ASSERT_TRUE(processor.processPacket(container));
    TEST_ASSERT_TRUE(spy.called);
    TEST_ASSERT_EQUAL_UINT32(sizeof(Packet), spy.last_length);
}

TEST_CASE("LedPacketProcessor rejects unknown packet sizes", "[packet]") {
    LedPacketProcessor processor;
    std::array<uint8_t, 5> payload = {1, 2, 3, 4, 5};
    GenericPacket packet(payload.data(), payload.size());

    TEST_ASSERT_FALSE(processor.canProcessPacket(packet));
    TEST_ASSERT_FALSE(processor.processPacket(packet));
}
