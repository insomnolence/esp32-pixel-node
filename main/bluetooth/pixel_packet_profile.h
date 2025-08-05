#ifndef PIXEL_PACKET_PROFILE_H_
#define PIXEL_PACKET_PROFILE_H_

#include "gatt_profile.h"
#include "../packet/generic_packet.h"
#include <functional>

class PixelPacketProfile : public GattProfile {
public:
    PixelPacketProfile(const std::string& service_uuid_str, const std::string& characteristic_uuid_str);
    ~PixelPacketProfile();

    void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    
    // Mesh integration callbacks
    void setBleConnectionCallback(std::function<void(bool)> callback);
    void setPacketForwardCallback(std::function<void(const GenericPacket&)> callback);

private:
    void handleReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleCreateServiceEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleDisconnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleAddCharDescrEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    
    // Mesh integration callbacks
    std::function<void(bool)> ble_connection_callback;
    std::function<void(const GenericPacket&)> packet_forward_callback;
};

#endif // PIXEL_PACKET_PROFILE_H_