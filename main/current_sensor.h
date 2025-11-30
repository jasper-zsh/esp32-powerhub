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
    float channel_currents[CURRENT_SENSOR_MAX_CHANNELS]; // 各通道电流 (A)，无数据填-1
    uint32_t timestamp_ms;                         // 采集时间戳
    uint32_t valid_mask;                           // bit1-6对应CH1-CH6
} current_sensor_data_t;

typedef struct {
    float acs712_offset;       // 通道传感器零点 (V)
    float acs712_sensitivity;  // 通道传感器灵敏度 (V/A)
} current_sensor_calibration_t;

typedef struct {
    float offset;       // 零点电压 (V)
    float sensitivity;  // 灵敏度 (V/A)
    int8_t adc_channel; // -1 表示禁用
} current_sensor_params_config_t;

typedef struct {
    current_sensor_params_config_t *channel_current;    // 动态数组，长度为channel_count
    uint8_t channel_count;                              // 实际电流通道数量
    uint32_t sample_interval_ms;                        // 采样间隔
} current_sensor_config_t;

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

// 读取电源电压（毫伏）；若不可用返回ESP_ERR_NOT_SUPPORTED
esp_err_t current_sensor_get_power_voltage_mv(uint16_t *out_mv);

// 动态配置辅助接口（可选）
esp_err_t current_sensor_config_clone(current_sensor_config_t *config_out);
void current_sensor_config_free(current_sensor_config_t *config);
esp_err_t current_sensor_apply_config(const current_sensor_config_t *config);

// BLE桥接接口
struct ble_gatt_access_ctxt;
int current_sensor_ble_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt);
void current_sensor_ble_subscribe(uint16_t conn_handle, uint16_t attr_handle, bool enabled);

#ifdef __cplusplus
}
#endif
