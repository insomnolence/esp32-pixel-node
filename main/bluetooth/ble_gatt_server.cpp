#include "ble_gatt_server.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"

const char* BLEGattServer::TAG = "BLEGattServer";

// Static member to hold the instance pointer
BLEGattServer* BLEGattServer::instance = nullptr;

BLEGattServer::BLEGattServer() {
    adv_params = {
        .adv_int_min = 0x20,
        .adv_int_max = 0x40,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    instance = this; // Store the instance pointer
}

BLEGattServer::~BLEGattServer() {
    instance = nullptr;
}

esp_err_t BLEGattServer::bleControllerInitAndEnable() {
    // In ESP-IDF 5.x, BLE controller initialization is handled automatically
    // by the Bluedroid stack - no manual controller init needed for BLE-only
    ESP_LOGI(TAG, "BLE controller initialization skipped - handled by Bluedroid");
    return ESP_OK;
}

esp_err_t BLEGattServer::bleBluedroidInitAndEnable() {
    esp_err_t ret;
    
    // Check if Bluedroid is already initialized (may happen with WiFi coexistence)
    esp_bt_controller_status_t bt_state = esp_bt_controller_get_status();
    if (bt_state == ESP_BT_CONTROLLER_STATUS_IDLE) {
        ESP_LOGI(TAG, "BT controller not initialized, initializing now");
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret) {
            ESP_LOGE(TAG, "%s bt controller init failed: %s", __func__, esp_err_to_name(ret));
            return ret;
        }
        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret) {
            ESP_LOGE(TAG, "%s bt controller enable failed: %s", __func__, esp_err_to_name(ret));
            return ret;
        }
    } else {
        ESP_LOGI(TAG, "BT controller already initialized (likely by WiFi coexistence)");
    }
    
    // Check if Bluedroid is already initialized
    esp_bluedroid_status_t bluedroid_status = esp_bluedroid_get_status();
    if (bluedroid_status == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        ret = esp_bluedroid_init();
        if (ret) {
            ESP_LOGE(TAG, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
            return ret;
        }
    } else {
        ESP_LOGI(TAG, "Bluedroid already initialized");
    }
    
    if (bluedroid_status != ESP_BLUEDROID_STATUS_ENABLED) {
        ret = esp_bluedroid_enable();
        if (ret) {
            ESP_LOGE(TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
            return ret;
        }
    } else {
        ESP_LOGI(TAG, "Bluedroid already enabled");
    }
    
    return ESP_OK;
}

esp_err_t BLEGattServer::init() {
    esp_err_t ret;
    ret = bleControllerInitAndEnable();
    if (ret != ESP_OK) return ret;

    ret = bleBluedroidInitAndEnable();
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t BLEGattServer::registerGattCallbacks() {
    ESP_LOGI(TAG, "🔥 Registering GATT callbacks");
    esp_err_t ret = esp_ble_gatts_register_callback(BLEGattServer::gattsEventHandler);
    if (ret) {
        ESP_LOGE(TAG, "🔥 gatts register error, error code = %x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "🔥 GATT callbacks registered successfully");
    return ESP_OK;
}

esp_err_t BLEGattServer::registerGattApp(uint16_t app_id) {
    ESP_LOGI(TAG, "🔥 Registering GATT app with ID: %d", app_id);
    esp_err_t ret = esp_ble_gatts_app_register(app_id);
    if (ret) {
        ESP_LOGE(TAG, "🔥 gatts app register error, error code = %x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "🔥 GATT app registration initiated successfully");
    return ESP_OK;
}

esp_err_t BLEGattServer::startAdvertising() {
    esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret) {
        ESP_LOGE(TAG, "Advertising start failed");
        return ret;
        }
    ESP_LOGI(TAG, "Advertising started");
    return ESP_OK;
}

esp_err_t BLEGattServer::setDeviceName(const char *device_name){
    esp_err_t ret = esp_ble_gap_set_device_name(device_name);
      if (ret)
    {
        ESP_LOGE(TAG, "set device name failed, error code = %x", ret);
        return ret;
    }
    return ESP_OK;
}

void BLEGattServer::gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (BLEGattServer::instance == nullptr) {
        ESP_LOGE(BLEGattServer::TAG, "BLEGattServer instance is null!");
        return;
    }
    BLEGattServer::instance->handleGattsEvent(event, gatts_if, param);
}

void BLEGattServer::handleGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(BLEGattServer::TAG, "Event: %d, gatts_if: %d", event, gatts_if);
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            // Find the appropriate profile
            for (auto&  profile : profile_list) {
                if (profile->getAppId() == param->reg.app_id) {
                    profile->setGattIf(gatts_if);
                    break;
                }            
            }
        } else {
            ESP_LOGE(BLEGattServer::TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile, call profile handler,
     * Call each profile's callback */
    do {
        for (auto&  profile : profile_list) {
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == profile->getGattIf()) {
                profile->gattsEventHandler(event, gatts_if, param);
            }
        }
    } while (0);
}

void BLEGattServer::addProfile(std::shared_ptr<GattProfile> profile) {
    ESP_LOGI(BLEGattServer::TAG, "Adding profile: %d", profile->getAppId());
    profile_list.push_back(profile);

    // This order matters. Configure Advertising Data MUSTT happen before resgisterGattApp.
    profile->configureAdvertisingData();

    if (registerGattApp(profile->getAppId()) != ESP_OK) {
        ESP_LOGE(BLEGattServer::TAG, "Failed to register GATT application for PixelPacketProfile");
    }

    // Set BLE Advertising callback
    profile->setAdvertisingCallback([this]() {
        this->startAdvertising();
    });
}

void BLEGattServer::setAdvertisingCallback(std::function<void()> callback) {
    advertisingCallback = callback;
}


