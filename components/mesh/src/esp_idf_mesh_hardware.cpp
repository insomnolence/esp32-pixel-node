#include "mesh/esp_idf_mesh_hardware.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "mesh/mesh_protocol.h" // For ESPNOW_MESH_CHANNEL

const char* EspIdfMeshHardware::TAG = "EspIdfMeshHardware";
MeshHardwareInterface::ReceiveCallback* EspIdfMeshHardware::recv_cb_ptr = nullptr;

EspIdfMeshHardware::EspIdfMeshHardware() {
    // Constructor
}

void EspIdfMeshHardware::espNowRecvWrapper(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (recv_cb_ptr) {
        recv_cb_ptr(recv_info->src_addr, data, len, recv_info->rx_ctrl->rssi);
    }
}

void EspIdfMeshHardware::setReceiveCallback(ReceiveCallback* cb) {
    recv_cb_ptr = cb;
    if (cb) {
        esp_now_register_recv_cb(espNowRecvWrapper);
    } else {
        esp_now_unregister_recv_cb();
    }
}

esp_err_t EspIdfMeshHardware::initWiFi() {
    // Standard ESP-IDF WiFi initialization for ESP-NOW
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) return err;

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { // Ignore if already created
         return err;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) return err;

    err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) return err;

    err = esp_wifi_start();
    if (err != ESP_OK) return err;

    err = esp_wifi_set_channel(ESPNOW_MESH_CHANNEL, WIFI_SECOND_CHAN_NONE);
    return err;
}

esp_err_t EspIdfMeshHardware::initESPNow() {
    return esp_now_init();
}

esp_err_t EspIdfMeshHardware::stopESPNow() {
    return esp_now_deinit();
}

esp_err_t EspIdfMeshHardware::sendEspNow(const uint8_t* dest_mac, const uint8_t* data, size_t len) {
    return esp_now_send(dest_mac, data, len);
}

esp_err_t EspIdfMeshHardware::addPeer(const uint8_t* peer_mac, uint8_t channel, bool encrypt) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, peer_mac, 6);
    peer.channel = channel;
    peer.encrypt = encrypt;
    // peer.ifidx is default (WIFI_IF_STA) which is usually 0
    return esp_now_add_peer(&peer);
}

esp_err_t EspIdfMeshHardware::getMacAddress(uint8_t* mac_out) {
    return esp_wifi_get_mac(WIFI_IF_STA, mac_out);
}

uint32_t EspIdfMeshHardware::getMillis() {
    return esp_timer_get_time() / 1000;
}

uint32_t EspIdfMeshHardware::getMicros() {
    return esp_timer_get_time();
}

uint32_t EspIdfMeshHardware::getRandom() {
    return esp_random();
}
