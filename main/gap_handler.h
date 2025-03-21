#ifndef GAP_HANDLER_H_
#define GAP_HANDLER_H_

#include "esp_err.h"
#include "esp_gap_ble_api.h"

esp_err_t register_gap_callbacks();
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

#endif // GAP_HANDLER_H_