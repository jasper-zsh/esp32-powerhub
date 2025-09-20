#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the WS2812 RGB LED driver
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t led_rgb_init(void);

/**
 * @brief Set the RGB color of the WS2812 LED
 * 
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t led_rgb_set_color(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Turn off the RGB LED
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t led_rgb_off(void);

/**
 * @brief Set LED to a predefined color for system status
 * 
 * @param is_connected Bluetooth connection status
 * @param has_error Error status
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t led_rgb_set_status(bool is_connected, bool has_error);

#ifdef __cplusplus
}
#endif
