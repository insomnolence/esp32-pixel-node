#pragma once

#include "mesh/mesh_hardware_interface.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "esp_log.h"
#include <cstring>

/**
 * @brief Concrete implementation of MeshHardwareInterface using ESP-IDF APIs.
 */
class EspIdfMeshHardware : public MeshHardwareInterface {
public:
    EspIdfMeshHardware();
    virtual ~EspIdfMeshHardware() = default;

    esp_err_t initWiFi() override;
    esp_err_t initESPNow() override;
    esp_err_t stopESPNow() override;

    esp_err_t sendEspNow(const uint8_t* dest_mac, const uint8_t* data, size_t len) override;
    esp_err_t addPeer(const uint8_t* peer_mac, uint8_t channel, bool encrypt) override;

    esp_err_t getMacAddress(uint8_t* mac_out) override;
    uint32_t getMillis() override;
    uint32_t getMicros() override;
    uint32_t getRandom() override;
    
    void setReceiveCallback(ReceiveCallback* cb) override;

private:
    static const char* TAG;
    static ReceiveCallback* recv_cb_ptr;
    static void espNowRecvWrapper(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
};
