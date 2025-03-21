#include "ble_gatt_server.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"

const char* BLEGattServer::TAG = "BLEGattServer";

GattsProfileInst::GattsProfileInst() {
    // Constructor
    gatts_cb = nullptr;
    gatts_if = ESP_GATT_IF_NONE;
}

GattsProfileInst::~GattsProfileInst() {
    // Destructor (if needed for cleanup)
}

void GattsProfileInst::setGattCallback(std::function<void(esp_gatts_cb_event_t, esp_gatt_if_t, esp_ble_gatts_cb_param_t*)> callback) {
    gatts_cb = callback;
}

void GattsProfileInst::setGattIf(esp_gatt_if_t gatts_if) {
    this->gatts_if = gatts_if;
}

std::function<void(esp_gatts_cb_event_t, esp_gatt_if_t, esp_ble_gatts_cb_param_t*)> GattsProfileInst::getGattCallback() {
    return gatts_cb;
}

esp_gatt_if_t GattsProfileInst::getGattIf() {
    return gatts_if;
}

BLEGattServer::BLEGattServer() {
    // Constructor
    // Initialize the gl_profile_tab array in the constructor
    for (int i = 0; i < PROFILE_NUM; ++i) {
        gl_profile_tab[i].setGattCallback(nullptr); // Initialize with null callbacks
        gl_profile_tab[i].setGattIf(ESP_GATT_IF_NONE);
    }
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

esp_err_t BLEGattServer::setDeviceName(const char *device_name){
    esp_err_t ret = esp_ble_gap_set_device_name(device_name);
      if (ret)
    {
        ESP_LOGE(TAG, "set device name failed, error code = %x", ret);
        return ret;
    }
    return ESP_OK;
}

void BLEGattServer::setProfileEventHandler(std::function<void(esp_gatts_cb_event_t, esp_gatt_if_t, esp_ble_gatts_cb_param_t*)> eventHandler, uint8_t profile_id) { // Change here
    if (profile_id < PROFILE_NUM) {
        gl_profile_tab[profile_id].setGattCallback(eventHandler);
    } else {
        ESP_LOGE(TAG, "Invalid profile ID: %u", profile_id);
    }
}
// Static member to hold the instance pointer
BLEGattServer* BLEGattServer::instance = nullptr;

void BLEGattServer::gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (BLEGattServer::instance == nullptr) {
        ESP_LOGE(BLEGattServer::TAG, "BLEGattServer instance is null!");
        return;
    }
    BLEGattServer::instance->handleGattsEvent(event, gatts_if, param);
}

void BLEGattServer::handleGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].setGattIf(gatts_if);
        } else {
            ESP_LOGE(BLEGattServer::TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE ||
                gatts_if == gl_profile_tab[idx].getGattIf()) {
                if (gl_profile_tab[idx].getGattCallback()) {
                    gl_profile_tab[idx].getGattCallback()(event, gatts_if, param);
                }
            }
        }
    } while (0);
}
