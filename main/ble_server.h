#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ble_server_init(void);

/**
 * @brief Check and send channel state notifications if states have changed
 *
 * This function should be called after PWM channel states are updated
 * to notify subscribed clients of any state changes.
 */
void ble_server_check_state_notifications(void);

#ifdef __cplusplus
}
#endif

