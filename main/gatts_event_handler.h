#ifndef GATTS_EVENT_HANDLER_H_
#define GATTS_EVENT_HANDLER_H_

#include "esp_gatts_api.h"
//#include "esp_gatt_if.h"

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#endif // GATTS_EVENT_HANDLER_H_

