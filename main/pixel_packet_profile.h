#ifndef PIXEL_PACKET_PROFILE_H_
#define PIXEL_PACKET_PROFILE_H_

#include "gatt_profile.h"

class PixelPacketProfile : public GattProfile {
public:
    PixelPacketProfile(const std::string& service_uuid_str, const std::string& characteristic_uuid_str);
    ~PixelPacketProfile();

    void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;

private:
    void handleReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleCreateServiceEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleDisconnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
    void handleAddCharDescrEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override;
};

#endif // PIXEL_PACKET_PROFILE_H_