#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 全局PWM硬件定义
#define PWM_CHANNEL_COUNT 6

// 温度传感器硬件定义
#define TEMP_SENSOR_COUNT 2
#define TEMP_SENSOR_GPIO_NUM 7

extern const int PWM_CHANNEL_GPIOS[PWM_CHANNEL_COUNT];

#define EXTERNAL_ADC
#define EXTERNAL_ADC_CS_PIN 3
#define EXTERNAL_ADC_SCLK_PIN 4
#define EXTERNAL_ADC_MISO_PIN 5
#define EXTERNAL_ADC_MOSI_PIN 6
// #define BUILTIN_ADC

// 电流传感器通道，-1为没有该传感器
#define TOTAL_CURRENT_CH 0
#define TOTAL_CURRENT_OFFSET 2.5f
#define TOTAL_CURRENT_SENSITIVITY 0.020f

extern const int CHANNEL_CURRENT_ADC_CHS[PWM_CHANNEL_COUNT];
#define CHANNEL_CURRENT_OFFSET 2.5f
#define CHANNEL_CURRENT_SENSITIVITY 0.100f

#ifdef __cplusplus
}
#endif
