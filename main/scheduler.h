#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t scheduler_init(void);

esp_err_t scheduler_submit_command(const control_cmd_t *cmd, TickType_t ticks_to_wait);


// 获取通道运行态快照
esp_err_t scheduler_get_channel_snapshot(uint8_t ch, control_cmd_t *out_cmd);

// 恢复通道运行态
esp_err_t scheduler_restore_channel_snapshot(uint8_t ch, const control_cmd_t *cmd);

#ifdef __cplusplus
}
#endif
