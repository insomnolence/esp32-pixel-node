#include "ble_gatt_server.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"

const char* BLEGattServer::TAG = "BLEGattServer";

// Static member to hold the instance pointer
BLEGattServer* BLEGattServer::instance = nullptr;

BLEGattServer::BLEGattServer() {
    // Constructor

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
    // Destructor (if needed for cleanup)
    instance = nullptr;
}

esp_err_t BLEGattServer::bleControllerInitAndEnable() {
    esp_err_t ret;
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t BLEGattServer::bleBluedroidInitAndEnable() {
    esp_err_t ret;
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ret;
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
    esp_err_t ret = esp_ble_gatts_register_callback(BLEGattServer::gattsEventHandler);
    if (ret) {
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return ret;
    }
    return ESP_OK;
}

esp_err_t BLEGattServer::registerGattApp(uint16_t app_id) {
    esp_err_t ret = esp_ble_gatts_app_register(app_id);
    if (ret) {
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return ret;
    }
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


