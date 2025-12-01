#include "unity.h"
#include "bluetooth/ble_gatt_server.h"
#include "mock_ble_hardware.h"
#include "bluetooth/gatt_profile.h"

// Mock Profile for testing
class MockGattProfile : public GattProfile {
public:
    MockGattProfile(const std::string& service_uuid, const std::string& char_uuid)
        : GattProfile(service_uuid, char_uuid) {}

    bool eventHandlerCalled = false;
    esp_gatts_cb_event_t lastEvent;

    void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) override {
        eventHandlerCalled = true;
        lastEvent = event;
    }
};

TEST_CASE("BLEGattServer initializes stack correctly", "[bluetooth]") {
    MockBleHardware mockHw;
    BLEGattServer server(&mockHw);

    TEST_ASSERT_EQUAL(ESP_OK, server.init());
    TEST_ASSERT_TRUE(mockHw.initStackCalled);
}

TEST_CASE("BLEGattServer sets device name and advertising", "[bluetooth]") {
    MockBleHardware mockHw;
    BLEGattServer server(&mockHw);

    TEST_ASSERT_EQUAL(ESP_OK, server.setDeviceName("Test Device"));
    TEST_ASSERT_EQUAL_STRING("Test Device", mockHw.deviceName.c_str());

    TEST_ASSERT_EQUAL(ESP_OK, server.startAdvertising());
    TEST_ASSERT_TRUE(mockHw.startAdvertisingCalled);
}

TEST_CASE("BLEGattServer registers app and handles events", "[bluetooth]") {
    MockBleHardware mockHw;
    BLEGattServer server(&mockHw);

    // Register callbacks first
    TEST_ASSERT_EQUAL(ESP_OK, server.registerGattCallbacks());
    TEST_ASSERT_NOT_NULL(mockHw.gattsCb);

    // Create and add a profile
    auto profile = std::make_shared<MockGattProfile>("00001800-0000-1000-8000-00805f9b34fb", "00002a00-0000-1000-8000-00805f9b34fb");
    
    // Adding profile should trigger registerGattsApp
    // Note: addProfile calls configureAdvertisingData and registerGattApp internally
    server.addProfile(profile);
    
    TEST_ASSERT_EQUAL(1, mockHw.registeredApps.size());
    TEST_ASSERT_EQUAL(profile->getAppId(), mockHw.registeredApps[0]);

    // Simulate REG_EVT to bind gatt_if to profile
    esp_ble_gatts_cb_param_t reg_param;
    reg_param.reg.status = ESP_GATT_OK;
    reg_param.reg.app_id = profile->getAppId();
    
    // Trigger event via hardware mock -> server static handler -> server instance -> profile
    mockHw.simulateGattsEvent(ESP_GATTS_REG_EVT, 3, &reg_param);

    // Verify profile received the event (it should have set gatt_if internally, usually not forwarded to handler for REG_EVT?
    // Let's check BLEGattServer::handleGattsEvent implementation.
    // It iterates profiles. If gatts_if matches, calls handler.
    // For REG_EVT, it sets profile->setGattIf(gatts_if).
    // Then it loops. gatts_if (3) == profile->getGattIf() (now 3). So it calls handler.
    
    TEST_ASSERT_TRUE(profile->eventHandlerCalled);
    TEST_ASSERT_EQUAL(ESP_GATTS_REG_EVT, profile->lastEvent);
    TEST_ASSERT_EQUAL(3, profile->getGattIf());
}
