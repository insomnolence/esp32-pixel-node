#ifndef LED_STRIP_ENCODER_H_
#define LED_STRIP_ENCODER_H_

#include "esp_err.h"
#include "driver/rmt_encoder.h"

/**
 * @brief Type of LED strip encoder configuration
 */
typedef struct {
    uint32_t resolution; ///< Encoder resolution, in Hz
} led_strip_encoder_config_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create RMT encoder for encoding LED strip pixels into RMT symbols
 *
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating LED strip encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_led_strip_encoder(const led_strip_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

#ifdef __cplusplus
}
#endif

#endif // LED_STRIP_ENCODER_H_