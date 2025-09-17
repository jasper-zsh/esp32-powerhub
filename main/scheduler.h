#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t scheduler_init(void);

esp_err_t scheduler_submit_command(const control_cmd_t *cmd, TickType_t ticks_to_wait);

esp_err_t scheduler_execute_preset(uint8_t preset_id, TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif
