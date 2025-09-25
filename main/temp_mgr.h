#ifndef TEMP_MGR_H
#define TEMP_MGR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// 温度管理器初始化
esp_err_t temp_mgr_init(void);

// 执行一次温度采样
esp_err_t temp_mgr_sample_once(int16_t *out_temp);

// 关闭温度传感器（用于深度睡眠前）
void temp_mgr_deinit(void);

#endif // TEMP_MGR_H
