#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_err.h"

// Forward declaration for ESP-NOW types to avoid heavy include dependency if possible,
// but since we need return types like esp_err_t, we might need some basic includes.
// For pure abstraction, we could wrap the error codes too, but keeping esp_err_t is pragmatic for ESP32 projects.

/**
 * @brief Abstract interface for hardware interactions required by the Mesh Coordinator.
 * 
 * This allows decoupling the mesh logic from specific ESP-IDF hardware calls,
 * enabling unit testing with mocks.
 */
class MeshHardwareInterface {
public:
    virtual ~MeshHardwareInterface() = default;

    // WiFi / ESP-NOW Initialization
    virtual esp_err_t initWiFi() = 0;
    virtual esp_err_t initESPNow() = 0;
    virtual esp_err_t stopESPNow() = 0;

    // ESP-NOW Operations
    virtual esp_err_t sendEspNow(const uint8_t* dest_mac, const uint8_t* data, size_t len) = 0;
    
    // Peer Management
    virtual esp_err_t addPeer(const uint8_t* peer_mac, uint8_t channel, bool encrypt) = 0;

    // System Utilities
    virtual esp_err_t getMacAddress(uint8_t* mac_out) = 0;
    virtual uint32_t getMillis() = 0;        // Replaces esp_timer_get_time() / 1000
    virtual uint32_t getMicros() = 0;        // Replaces esp_timer_get_time()
    virtual uint32_t getRandom() = 0;        // Replaces esp_random()
    
    // Callback abstraction
    // The coordinator provides a callback that the hardware layer calls when a packet arrives.
    // This avoids exposing ESP-IDF specific callback signatures to the coordinator.
    using ReceiveCallback = void(const uint8_t* mac, const uint8_t* data, int len, int8_t rssi);
    virtual void setReceiveCallback(ReceiveCallback* cb) = 0;
};
