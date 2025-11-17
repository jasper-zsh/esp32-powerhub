#ifndef TEMP_MGR_H
#define TEMP_MGR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#include "hardware_defs.h"

// 最大支持的DS18B20传感器数量
#define TEMP_MGR_MAX_SENSORS    TEMP_SENSOR_COUNT

// 温度传感器类型
typedef enum {
    TEMP_SENSOR_POWER = 0,    // 电源区域温度传感器
    TEMP_SENSOR_CONTROL = 1,  // 控制区域温度传感器
} temp_sensor_type_t;

// 温度管理器初始化
esp_err_t temp_mgr_init(void);

// 执行一次温度采样（指定传感器）
esp_err_t temp_mgr_sample_once(temp_sensor_type_t sensor_type, int16_t *out_temp);

// 执行一次温度采样（所有传感器）
esp_err_t temp_mgr_sample_all(int16_t out_temps[TEMP_SENSOR_COUNT]);

// 获取已连接的传感器数量
int temp_mgr_get_sensor_count(void);

// 关闭温度传感器（用于深度睡眠前）
void temp_mgr_deinit(void);

#endif // TEMP_MGR_H
