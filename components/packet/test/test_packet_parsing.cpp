#include "unity.h"
#include "packet/generic_packet.h"
#include "packet/packet.h"
#include <cstring>

TEST_CASE("GenericPacket constructor and data access", "[packet]") {
    const uint8_t test_data[] = {0x01, 0x02, 0x03};
    GenericPacket packet(test_data, sizeof(test_data));

    TEST_ASSERT_EQUAL_UINT32(sizeof(test_data), packet.getLength());
    TEST_ASSERT_EQUAL_MEMORY(test_data, packet.getData(), sizeof(test_data));
}

TEST_CASE("GenericPacket set/get typed packet", "[packet]") {
    struct TestStruct {
        uint32_t id;
        uint8_t value;
    } __attribute__((packed));

    TestStruct input = {0xAABBCCDD, 0x55};
    GenericPacket packet;
    
    TEST_ASSERT_TRUE(packet.setPacket(input));
    TEST_ASSERT_EQUAL_UINT32(sizeof(TestStruct), packet.getLength());

    TestStruct output;
    TEST_ASSERT_TRUE(packet.getPacket(output));
    TEST_ASSERT_EQUAL_UINT32(input.id, output.id);
    TEST_ASSERT_EQUAL_UINT8(input.value, output.value);
}

TEST_CASE("GenericPacket max size limit", "[packet]") {
    GenericPacket packet;
    uint8_t large_data[GenericPacket::MAX_PACKET_SIZE + 1];
    memset(large_data, 0, sizeof(large_data));

    TEST_ASSERT_FALSE(packet.setData(large_data, sizeof(large_data)));
    TEST_ASSERT_EQUAL_UINT32(0, packet.getLength());
}

TEST_CASE("Packet struct size", "[packet]") {
    // Verify Packet struct is packed correctly to 19 bytes
    TEST_ASSERT_EQUAL_UINT32(19, sizeof(Packet));
}

TEST_CASE("Packet serialization", "[packet]") {
    Packet input;
    memset(&input, 0, sizeof(Packet));
    input.command = 1;
    input.brightness = 255;
    input.color[0] = 0xFF0000; // Red
    
    GenericPacket generic;
    TEST_ASSERT_TRUE(generic.setPacket(input));
    TEST_ASSERT_EQUAL_UINT32(19, generic.getLength());
    
    Packet output;
    TEST_ASSERT_TRUE(generic.getPacket(output));
    TEST_ASSERT_EQUAL_UINT8(input.command, output.command);
    TEST_ASSERT_EQUAL_UINT8(input.brightness, output.brightness);
    TEST_ASSERT_EQUAL_UINT32(input.color[0], output.color[0]);
}
