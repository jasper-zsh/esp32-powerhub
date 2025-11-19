// 电流传感器抽象层
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "hardware_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CURRENT_SENSOR_MAX_CHANNELS PWM_CHANNEL_COUNT

typedef struct {
    float total_input_current;                     // 总输入电流 (A)，无数据填-1
    float channel_currents[CURRENT_SENSOR_MAX_CHANNELS]; // 各通道电流 (A)，无数据填-1
    uint32_t timestamp_ms;                         // 采集时间戳
    uint32_t valid_mask;                           // bit0=总电流，bit1-6对应CH1-CH6
} current_sensor_data_t;

typedef struct {
    float acs758_offset;       // 总电流传感器零点 (V)
    float acs758_sensitivity;  // 总电流传感器灵敏度 (V/A)
    float acs712_offset;       // 通道传感器零点 (V)
    float acs712_sensitivity;  // 通道传感器灵敏度 (V/A)
} current_sensor_calibration_t;

// 初始化（根据编译期配置选择驱动）
esp_err_t current_sensor_init(void);

// 启动后台采样任务
esp_err_t current_sensor_start(uint32_t interval_ms);

// 停止采样任务
esp_err_t current_sensor_stop(void);

// 读取最新数据
esp_err_t current_sensor_get_latest(current_sensor_data_t *data);

// 设置/获取校准
esp_err_t current_sensor_set_calibration(const current_sensor_calibration_t *cal);
esp_err_t current_sensor_get_calibration(current_sensor_calibration_t *cal);

// 重新采集零点
esp_err_t current_sensor_calibrate_zero(uint16_t sample_count);

// BLE桥接接口
struct ble_gatt_access_ctxt;
int current_sensor_ble_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt);
void current_sensor_ble_subscribe(uint16_t conn_handle, uint16_t attr_handle, bool enabled);

#ifdef __cplusplus
}
#endif
