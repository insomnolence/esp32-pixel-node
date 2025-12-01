#pragma once

#include "mesh/mesh_hardware_interface.h"
#include <vector>
#include <cstring>
#include <functional>

struct MockPacket {
    uint8_t dest_mac[6];
    std::vector<uint8_t> data;
};

struct MockPeer {
    uint8_t mac[6];
    uint8_t channel;
    bool encrypt;
};

class MockMeshHardware : public MeshHardwareInterface {
public:
    // State
    std::vector<MockPacket> sent_packets;
    std::vector<MockPeer> peers;
    uint8_t my_mac[6] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
    uint32_t current_millis = 1000;
    uint32_t current_micros = 1000000;
    std::vector<uint32_t> random_sequence = {123, 456, 789, 101112}; // Default sequence
    size_t random_index = 0;
    ReceiveCallback* recv_cb = nullptr;

    // Interface Implementation
    esp_err_t initWiFi() override { return ESP_OK; }
    esp_err_t initESPNow() override { return ESP_OK; }
    esp_err_t stopESPNow() override { return ESP_OK; }

    esp_err_t sendEspNow(const uint8_t* dest_mac, const uint8_t* data, size_t len) override {
        MockPacket p;
        memcpy(p.dest_mac, dest_mac, 6);
        p.data.assign(data, data + len);
        sent_packets.push_back(p);
        return ESP_OK;
    }

    esp_err_t addPeer(const uint8_t* peer_mac, uint8_t channel, bool encrypt) override {
        MockPeer p;
        memcpy(p.mac, peer_mac, 6);
        p.channel = channel;
        p.encrypt = encrypt;
        peers.push_back(p);
        return ESP_OK;
    }

    esp_err_t getMacAddress(uint8_t* mac_out) override {
        memcpy(mac_out, my_mac, 6);
        return ESP_OK;
    }

    uint32_t getMillis() override { return current_millis; }
    uint32_t getMicros() override { return current_micros; }
    uint32_t getRandom() override {
        if (random_sequence.empty()) return 0;
        uint32_t val = random_sequence[random_index];
        random_index = (random_index + 1) % random_sequence.size();
        return val;
    }

    void setReceiveCallback(ReceiveCallback* cb) override {
        recv_cb = cb;
    }

    // Helper methods for tests
    void setMac(const uint8_t* mac) {
        memcpy(my_mac, mac, 6);
    }

    void simulatePacketReceived(const uint8_t* src_mac, const uint8_t* data, size_t len, int8_t rssi) {
        if (recv_cb) {
            recv_cb(src_mac, data, len, rssi);
        }
    }
};
