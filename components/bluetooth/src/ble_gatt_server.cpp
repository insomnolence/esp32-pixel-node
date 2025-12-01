#include "bluetooth/ble_gatt_server.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"

const char* BLEGattServer::TAG = "BLEGattServer";

// Static member to hold the instance pointer
BLEGattServer* BLEGattServer::instance = nullptr;
SemaphoreHandle_t BLEGattServer::instance_mutex = nullptr;
uint32_t BLEGattServer::callback_counter = 0;

BLEGattServer::BLEGattServer(BleHardwareInterface* hardware)
    : hardware_(hardware) {
    adv_params = {
        .adv_int_min = 0x20,      // 20ms - fast advertising for quick discovery
        .adv_int_max = 0x30,      // 30ms - balanced for power and speed
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .peer_addr = {0},         // Not used for ADV_TYPE_IND
        .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,  // Not used for ADV_TYPE_IND
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    // Create mutex for thread-safe singleton access if not already created
    if (instance_mutex == nullptr) {
        instance_mutex = xSemaphoreCreateMutex();
        if (instance_mutex == nullptr) {
            ESP_LOGE(TAG, "Failed to create instance mutex");
        }
    }

    // Store the instance pointer with mutex protection
    if (instance_mutex && xSemaphoreTake(instance_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        instance = this;
        xSemaphoreGive(instance_mutex);
    } else {
        instance = this; // Fallback if mutex acquisition failed
    }

    ESP_LOGI(TAG, "BLE GATT Server configured with optimized advertising parameters for faster connections");
}

BLEGattServer::~BLEGattServer() {
    // Clear instance pointer with mutex protection
    if (instance_mutex && xSemaphoreTake(instance_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        instance = nullptr;
        xSemaphoreGive(instance_mutex);
    } else {
        instance = nullptr;
    }

    // Clean up mutex when instance is destroyed
    if (instance_mutex != nullptr) {
        vSemaphoreDelete(instance_mutex);
        instance_mutex = nullptr;
    }
}

esp_err_t BLEGattServer::init() {
    if (!hardware_) {
        ESP_LOGE(TAG, "Hardware interface is null");
        return ESP_ERR_INVALID_ARG;
    }

    // Use hardware abstraction for BLE stack initialization
    esp_err_t ret = hardware_->initStack();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE stack: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t BLEGattServer::registerGattCallbacks() {
    ESP_LOGI(TAG, "🔥 Registering GATT callbacks");
    if (!hardware_) {
        ESP_LOGE(TAG, "Hardware interface is null");
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = hardware_->registerGattsCallback(BLEGattServer::gattsEventHandler);
    if (ret) {
        ESP_LOGE(TAG, "🔥 gatts register error, error code = %x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "🔥 GATT callbacks registered successfully");
    return ESP_OK;
}

esp_err_t BLEGattServer::registerGattApp(uint16_t app_id) {
    ESP_LOGI(TAG, "🔥 Registering GATT app with ID: %d", app_id);
    if (!hardware_) {
        ESP_LOGE(TAG, "Hardware interface is null");
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = hardware_->registerGattsApp(app_id);
    if (ret) {
        ESP_LOGE(TAG, "🔥 gatts app register error, error code = %x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "🔥 GATT app registration initiated successfully");
    return ESP_OK;
}

esp_err_t BLEGattServer::startAdvertising() {
    ESP_LOGI(TAG, "🔍 Attempting to start BLE advertising...");
    if (!hardware_) {
        ESP_LOGE(TAG, "Hardware interface is null");
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = hardware_->startAdvertising(&adv_params);
    if (ret) {
        ESP_LOGE(TAG, "🚨 Advertising start FAILED: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "🔍 BLE advertising started successfully");
    return ESP_OK;
}

esp_err_t BLEGattServer::setDeviceName(const char *device_name){
    if (!hardware_) {
        ESP_LOGE(TAG, "Hardware interface is null");
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = hardware_->setDeviceName(device_name);
    if (ret) {
        ESP_LOGE(TAG, "set device name failed, error code = %x", ret);
        return ret;
    }
    return ESP_OK;
}

void BLEGattServer::gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    callback_counter++; // Increment callback counter for debugging
    ESP_LOGI(BLEGattServer::TAG, "GATTS Event Handler Called: event=%d, gatts_if=%d, count=%lu", event, gatts_if, callback_counter);

    // Thread-safe access to singleton instance with short timeout to prevent system hangs
    if (instance_mutex && xSemaphoreTake(instance_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (instance) {
            ESP_LOGI(BLEGattServer::TAG, "Forwarding to handleGattsEvent");
            instance->handleGattsEvent(event, gatts_if, param);
        } else {
            ESP_LOGE(BLEGattServer::TAG, "BLEGattServer instance is null! Event=%d ignored", event);
        }
        xSemaphoreGive(instance_mutex);
    } else {
        ESP_LOGE(BLEGattServer::TAG, "Failed to acquire instance mutex! Event=%d ignored", event);
    }
}

void BLEGattServer::handleGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(BLEGattServer::TAG, "🔍 handleGattsEvent: Event=%d, gatts_if=%d", event, gatts_if);
    
    // Log common connection events in detail
    switch(event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "🔍 ESP_GATTS_REG_EVT: App registration event");
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "🔥🔥 ESP_GATTS_CONNECT_EVT: BLE CLIENT CONNECTION DETECTED!");
            ESP_LOGI(TAG, "🔥 Connection ID: %d", param->connect.conn_id);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "🔥 ESP_GATTS_DISCONNECT_EVT: BLE CLIENT DISCONNECTED");
            break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "🔍 ESP_GATTS_WRITE_EVT: Data write received");
            break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "🔍 ESP_GATTS_READ_EVT: Data read requested");
            break;
        default:
            ESP_LOGI(TAG, "🔍 Other GATTS event: %d", event);
            break;
    }
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

void BLEGattServer::validateGattServiceHealth() {
    ESP_LOGI(TAG, "🔍 === GATT SERVICE HEALTH CHECK ===");
    
    // Check instance pointer
    ESP_LOGI(TAG, "🔍 BLEGattServer instance: %s", (instance != nullptr) ? "VALID" : "NULL");
    
    // Check profile list
    ESP_LOGI(TAG, "🔍 Registered profiles: %zu", profile_list.size());
    
    // Check Bluedroid status
    esp_bluedroid_status_t bt_status = esp_bluedroid_get_status();
    ESP_LOGI(TAG, "🔍 Bluedroid status: %d (0=uninitialized, 1=initialized, 2=enabled)", bt_status);
    
    // Test if we can trigger a dummy GATT event (this should help identify callback corruption)
    ESP_LOGI(TAG, "🔍 Testing GATT event handler accessibility...");
    if (instance != nullptr) {
        ESP_LOGI(TAG, "🔍 Instance accessible - GATT callbacks should work");
    } else {
        ESP_LOGE(TAG, "🔍 ❌ Instance is NULL - GATT callbacks WILL FAIL!");
    }
    
    ESP_LOGI(TAG, "🔍 === END GATT HEALTH CHECK ===");
}

void BLEGattServer::staticValidateGattHealth() {
    ESP_LOGI(TAG, "🔍 === STATIC GATT HEALTH CHECK ===");
    
    // Check static instance pointer
    ESP_LOGI(TAG, "🔍 Static instance pointer: %s", (instance != nullptr) ? "VALID" : "NULL");
    ESP_LOGI(TAG, "🔍 Total GATT callbacks received: %lu", callback_counter);
    
    // Check Bluedroid status
    esp_bluedroid_status_t bt_status = esp_bluedroid_get_status();
    ESP_LOGI(TAG, "🔍 Bluedroid status: %d (0=uninit, 1=init, 2=enabled)", bt_status);
    
    // If instance exists, call the full health check
    if (instance != nullptr) {
        ESP_LOGI(TAG, "🔍 Instance found - calling full health check");
        instance->validateGattServiceHealth();
    } else {
        ESP_LOGE(TAG, "🔍 ❌ CRITICAL: Instance is NULL - GATT callbacks WILL NOT WORK!");
        ESP_LOGE(TAG, "🔍 ❌ This explains why BLE connections aren't generating events!");
    }
    
    ESP_LOGI(TAG, "🔍 === END STATIC GATT HEALTH CHECK ===");
}

