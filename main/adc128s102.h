// ADC128S102外置ADC驱动
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADC128S102_CHANNEL_COUNT    8
#define ADC128S102_MAX_CHANNELS     6   // 实际使用通道数 (IN0总电流, IN1-IN6各通道电流)

// 通道定义
typedef enum {
    ADC_CHANNEL_TOTAL_CURRENT = 0,   // IN0: 总输入电流 (ACS758)
    ADC_CHANNEL_CH1_CURRENT = 1,     // IN1: CH1电流 (ACS712)
    ADC_CHANNEL_CH2_CURRENT = 2,     // IN2: CH2电流 (ACS712)
    ADC_CHANNEL_CH3_CURRENT = 3,     // IN3: CH3电流 (ACS712)
    ADC_CHANNEL_CH4_CURRENT = 4,     // IN4: CH4电流 (ACS712)
    ADC_CHANNEL_CH5_CURRENT = 5,     // IN5: CH5电流 (ACS712)
    ADC_CHANNEL_CH6_CURRENT = 6,     // IN6: CH6电流 (ACS712)
    ADC_CHANNEL_RESERVED = 7,        // IN7: 保留
} adc_channel_t;

// 电流传感器数据结构
typedef struct {
    float total_input_current;       // 总输入电流 (A)
    float channel_currents[6];       // 各通道电流 (A)
    uint32_t timestamp;              // 采集时间戳 (ms)
} current_sensor_data_t;

// 校准参数
typedef struct {
    float acs758_offset;             // ACS758零点偏置 (V)
    float acs758_sensitivity;        // ACS758灵敏度 (V/A)
    float acs712_offset;             // ACS712零点偏置 (V)
    float acs712_sensitivity;        // ACS712灵敏度 (V/A)
    bool enable_filter;              // 是否启用滤波
} adc_calibration_t;

// 初始化ADC128S102
esp_err_t adc128s102_init(void);

// 读取单个通道ADC原始值
esp_err_t adc128s102_read_raw(uint8_t channel, uint16_t *raw_value);

// 读取单个通道电流值 (经过校准和转换)
esp_err_t adc128s102_read_current(uint8_t channel, float *current);

// 批量读取所有电流数据
esp_err_t adc128s102_read_all_currents(current_sensor_data_t *data);

// 设置校准参数
esp_err_t adc128s102_set_calibration(const adc_calibration_t *cal);

// 获取校准参数
esp_err_t adc128s102_get_calibration(adc_calibration_t *cal);

// 启动连续采集任务
esp_err_t adc128s102_start_continuous_sampling(uint32_t interval_ms);

// 停止连续采集任务
esp_err_t adc128s102_stop_continuous_sampling(void);

// 获取最新的电流数据
esp_err_t adc128s102_get_latest_data(current_sensor_data_t *data);

// 零点校准
esp_err_t adc128s102_calibrate_zero_point(void);

// BLE接口 - 前向声明避免头文件依赖
struct ble_gatt_access_ctxt;
int adc128s102_ble_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt);
void adc128s102_ble_subscribe(uint16_t conn_handle, uint16_t attr_handle, bool enabled);

#ifdef __cplusplus
}
#endif