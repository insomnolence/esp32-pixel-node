#include "unity.h"
#include "bluetooth/ble_gap_handler.h"
#include "mock_ble_hardware.h"

TEST_CASE("BLEGapHandler registers callback", "[bluetooth]") {
    MockBleHardware mockHw;
    BLEGapHandler gapHandler(&mockHw);

    TEST_ASSERT_EQUAL(ESP_OK, gapHandler.registerGapCallbacks());
    TEST_ASSERT_NOT_NULL(mockHw.gapCb);
}

TEST_CASE("BLEGapHandler handles events (smoke test)", "[bluetooth]") {
    MockBleHardware mockHw;
    BLEGapHandler gapHandler(&mockHw);
    gapHandler.registerGapCallbacks();

    // Simulate an event to ensure no crash and coverage of switch case
    esp_ble_gap_cb_param_t param;
    param.adv_start_cmpl.status = ESP_BT_STATUS_SUCCESS;
    
    // Trigger the static callback via the mock
    if (mockHw.gapCb) {
        mockHw.gapCb(ESP_GAP_BLE_ADV_START_COMPLETE_EVT, &param);
    }
    
    // Since the handler is currently void return and just logs, 
    // we mostly verify it doesn't crash. 
    // In a more advanced test framework, we'd mock ESP_LOG to verify output.
}
